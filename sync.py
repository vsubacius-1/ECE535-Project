#!/usr/bin/env python3
"""Clock synchronization for the two ESP32 nodes, from the CSV that ble_rx.py writes.

The result is the clock offset between the nodes: offset = t_sound - t_acu, in us,
for one physical instant. Drift is the rate of change of the offset, in ppm (us/s).

Four methods get the same data, so you can compare them:

  haest        Baseline. HAEST (RTAS 2024) Section V-A: k-means on event offsets,
               mean of the cluster slopes, offset of the cluster with the smallest mean.
  packets      Packet timing only: the lower envelope of the BLE delay. No events.
  ours_events  Our Kalman filter, events only.
  ours_hybrid  Our Kalman filter. The packets give the drift, the events give the offset.

Usage:
  python3 sync.py run.csv --bias-us -1100                  # summary of all methods
  python3 sync.py run.csv --bias-us -1100 --truth truth.csv --plot sync.png --out est.csv
  python3 sync.py run.csv --pairing pattern                # pair events without packet timing
  python3 sync.py run.csv --follow                         # live, while ble_rx.py writes run.csv

--bias-us is the calibrated (sound delay - ACU delay) for your reference tap position.
Measure it once with the logic analyzer. See SYNC.md.

Truth: if the CSV has truth-pulse packets (trigger = 2, from truth_pulse.py and the
v3/v2 sketches), sync.py pairs them and uses them as truth automatically. Pulse
packets are never used as events. --truth FILE overrides them.

Needs numpy. --plot also needs matplotlib.
"""

import argparse
import bisect
import csv
import math
import sys
import time
from collections import deque, namedtuple
from dataclasses import dataclass, fields

import numpy as np

NODE_ACU = 1
NODE_SOUND = 2
NODES = (NODE_ACU, NODE_SOUND)
METHODS = ("haest", "packets", "ours_events", "ours_hybrid")

# One CSV row from ble_rx.py. pi_us is the Pi monotonic clock in us.
Packet = namedtuple("Packet", "pi_us node seq t_edge tx level trigger")
# One detected event. t_pi is Pi time in s from the start of the run.
Event = namedtuple("Event", "node t_pi t_edge level")


@dataclass
class Config:
    # Event detection and pairing
    holdoff_ms: float = 250.0        # quiet time that separates two events on one node
    pair_window_ms: float = 50.0     # max Pi-time difference of a sound/ACU pair (start)
    pair_window_narrow_ms: float = 10.0  # after pair_narrow_after pairs, centred on their median
    pair_narrow_after: int = 5
    resolve_margin_s: float = 0.5    # wait this long for late packets before pairing
    coarse_bin_s: float = 2.0        # envelope bin for the coarse node -> Pi mapping
    pattern_window_s: float = 120.0  # --pairing pattern: window length
    pattern_tol_us: float = 10000.0  # --pairing pattern: match tolerance
    truth_match_s: float = 0.4       # pair truth pulses from the two nodes within this
    # Calibration
    bias_us: float = 0.0             # (sound delay - ACU delay) of the reference group
    # Kalman filter
    kf_bin_s: float = 10.0           # one packet measurement per bin
    kf_bin_min_frac: float = 0.25    # skip a bin with fewer packets than this share
    sigma_pkt_us: float = 500.0      # noise of one packet-envelope measurement
    sigma_ev_us: float = 300.0       # noise of one event pair at normal amplitude
    q_theta: float = 1.0             # offset random walk, us^2/s
    q_alpha: float = 0.002           # drift random walk, ppm^2/s
    q_pkt: float = 20.0              # packet-bias random walk, us^2/s
    alpha0_ppm: float = 50.0         # prior drift uncertainty
    max_drift_ppm: float = 60.0      # events-only start: largest drift to consider
    bpkt0_us: float = 5000.0         # prior uncertainty of the packet bias
    gate: float = 3.0                # join a group if the innovation is < gate sigma
    group_eps_us: float = 500.0      # half-width of a new group
    group_min: int = 3               # events needed to confirm a new group
    pending_timeout_s: float = 1200.0  # forget unmatched events after this time
    reref_margin_us: float = 500.0   # switch reference if a group is this much lower
    use_weights: bool = True         # weight events by amplitude
    # HAEST baseline (values from the paper)
    haest_window_s: float = 600.0
    haest_k: int = 20
    haest_min_cluster: int = 3


# --------------------------------------------------------------------------
# CSV input
# --------------------------------------------------------------------------

def parse_row(rec):
    """Return a Packet from one CSV record, or None for a header or a bad line."""
    if len(rec) < 9:
        return None
    try:
        pi_ns, node, seq, t_edge, _t_peak, tx, _raw, level, trig = (int(v) for v in rec[:9])
    except ValueError:
        return None
    if node not in NODES:
        return None
    return Packet(pi_ns / 1000.0, node, seq, t_edge, tx, level, trig)


def load_csv(path):
    """Read a ble_rx.py CSV. Keep only data after the last reboot of each node."""
    with open(path, newline="") as f:
        rows = [p for p in (parse_row(r) for r in csv.reader(f)) if p is not None]
    rows.sort(key=lambda p: p.pi_us)
    last_tx = {}
    cut = {n: 0 for n in NODES}
    for i, p in enumerate(rows):
        if p.node in last_tx and p.tx < last_tx[p.node] - 1_000_000:
            cut[p.node] = i
        last_tx[p.node] = p.tx
    for n in NODES:
        if cut[n]:
            print(f"# node {n} rebooted, using only the data after the last reboot",
                  file=sys.stderr)
    return [p for i, p in enumerate(rows) if i >= cut[p.node]]


def load_truth(path):
    """Truth CSV: t_acu_us,t_sound_us = both node clocks at the same instant."""
    out = []
    with open(path, newline="") as f:
        for rec in csv.reader(f):
            try:
                out.append((int(rec[0]), int(rec[1])))
            except (ValueError, IndexError):
                continue
    return out


# --------------------------------------------------------------------------
# Building blocks
# --------------------------------------------------------------------------

class Envelope:
    """Max of u = tx - pi_rx in fixed Pi-time bins.

    u = (node clock - Pi clock) - BLE delay. The delay is always positive, so the
    max in a bin is the node offset minus the smallest delay.
    """

    def __init__(self, bin_s):
        self.bin_s = bin_s
        self.k = None
        self.cur = -math.inf
        self.n = 0
        self.prev = None  # (k, max) of the last complete bin

    def add(self, t, u):
        """Add one packet. Return (k, max, count) when a bin completes, else None."""
        k = int(t // self.bin_s)
        done = None
        if self.k is None:
            self.k = k
        if k != self.k:
            done = (self.k, self.cur, self.n)
            self.prev = (self.k, self.cur)
            self.k, self.cur, self.n = k, -math.inf, 0
        if u > self.cur:
            self.cur = u
        self.n += 1
        return done

    def value(self):
        v = self.cur
        if self.prev is not None and self.prev[0] == self.k - 1:
            v = max(v, self.prev[1])
        return v


class EventDetector:
    """One event per burst: a new event needs `holdoff` of quiet before it."""

    def __init__(self, holdoff_ms):
        self.holdoff_us = holdoff_ms * 1000.0
        self.last = None

    def is_new(self, t_edge):
        new = self.last is None or t_edge - self.last >= self.holdoff_us
        self.last = t_edge if self.last is None else max(self.last, t_edge)
        return new


class Pairer:
    """Pair ACU and sound events whose Pi times are within +/- W.

    A pair is kept only if it is unique both ways. If a window has two
    candidates, the event is discarded.
    """

    def __init__(self, cfg):
        self.w = cfg.pair_window_ms / 1000.0
        self.w_narrow = cfg.pair_window_narrow_ms / 1000.0
        self.narrow_after = cfg.pair_narrow_after
        self.margin = cfg.resolve_margin_s
        self.hist = {n: [] for n in NODES}
        self.todo = deque()
        self.recent = deque(maxlen=15)  # recent (sound - ACU) Pi-time differences
        self.ambiguous = 0
        self.unmatched = 0

    def _window(self):
        """(centre, half-width) of the search window for s.t_pi - a.t_pi."""
        if len(self.recent) >= self.narrow_after:
            return float(np.median(self.recent)), self.w_narrow
        return 0.0, self.w

    def add(self, ev):
        self.hist[ev.node].append(ev)
        if ev.node == NODE_ACU:
            self.todo.append(ev)

    def resolve(self, now):
        out = []
        while self.todo and self.todo[0].t_pi + self.w + self.margin <= now:
            a = self.todo.popleft()
            m, w = self._window()
            ss = [s for s in self.hist[NODE_SOUND] if abs(s.t_pi - a.t_pi - m) <= w]
            if len(ss) == 1:
                aa = [x for x in self.hist[NODE_ACU] if abs(ss[0].t_pi - x.t_pi - m) <= w]
                if len(aa) == 1:
                    out.append((a, ss[0]))
                    self.recent.append(ss[0].t_pi - a.t_pi)
                    continue
            if ss:
                self.ambiguous += 1
            else:
                self.unmatched += 1
        keep_after = now - (2 * self.w + self.margin + 2.0)
        for n in NODES:
            h = self.hist[n]
            if h and h[0].t_pi < keep_after:
                self.hist[n] = [e for e in h if e.t_pi >= keep_after]
        return out


def kmeans_1d(v, k, iters=50):
    """Plain 1-D k-means with quantile start. Returns (labels, centers)."""
    k = max(1, min(k, len(np.unique(v))))
    c = np.quantile(v, (np.arange(k) + 0.5) / k)
    labels = np.zeros(len(v), dtype=int)
    for _ in range(iters):
        labels = np.argmin(np.abs(v[:, None] - c[None, :]), axis=1)
        new = np.array([v[labels == j].mean() if np.any(labels == j) else c[j] for j in range(k)])
        if np.allclose(new, c):
            break
        c = new
    return labels, c


def ransac_line(t, z, eps, slope_lo, slope_hi):
    """Line z = z_i + slope * (t - t_i) through the most points within +/- eps,
    with slope_lo <= slope <= slope_hi.

    Tries every pair of points, and each point alone with the middle slope (fine
    for the < 100 points it gets). Returns (inlier indices, slope).
    """
    best, best_slope = np.array([], dtype=int), 0.0
    n = len(t)
    mid = 0.5 * (slope_lo + slope_hi)
    for i in range(n):
        cands = [mid] + [(z[j] - z[i]) / (t[j] - t[i]) for j in range(i + 1, n) if t[j] > t[i]]
        for slope in cands:
            if not slope_lo <= slope <= slope_hi:
                continue
            inl = np.nonzero(np.abs(z - z[i] - slope * (t - t[i])) <= eps)[0]
            if len(inl) > len(best):
                best, best_slope = inl, slope
    return best, best_slope


def densest_cluster(r, eps):
    """Indices of the largest set of values that fit in a band of width 2*eps."""
    order = np.argsort(r)
    rs = r[order]
    j = np.searchsorted(rs, rs + 2 * eps, side="right")
    counts = j - np.arange(len(rs))
    i = int(np.argmax(counts))
    return order[i:j[i]]


# --------------------------------------------------------------------------
# Estimators. All take packet(t, z) and event(t, z, sigma), and return
# estimate(t) -> dict(offset_us, drift_ppm, sigma_us, mode) or None.
# Time t is in s from the start of the run (Pi clock).
# --------------------------------------------------------------------------

class HaestBaseline:
    """HAEST Section V-A, on a rolling window of event pairs.

    1. Cluster the offsets with k-means (k = 20).  2. Drop clusters with < 3 points.
    3. Fit a slope per cluster; drift = mean slope.  4. Offset = mean of the cluster
    with the smallest mean. We extrapolate that offset to the query time with the
    drift (the paper does not say; this choice helps the baseline).
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.t = []
        self.z = []

    def packet(self, t, z):
        pass

    def event(self, t, z, sigma):
        i = bisect.bisect_right(self.t, t)
        self.t.insert(i, t)
        self.z.insert(i, z)
        return "added"

    def estimate(self, tq):
        c = self.cfg
        lo = bisect.bisect_left(self.t, tq - c.haest_window_s)
        hi = bisect.bisect_right(self.t, tq)
        if hi - lo < c.haest_min_cluster:
            return None
        t = np.array(self.t[lo:hi])
        z = np.array(self.z[lo:hi], dtype=float)
        labels, centers = kmeans_1d(z, c.haest_k)
        drifts, best = [], None
        for j in range(len(centers)):
            m = labels == j
            if m.sum() < c.haest_min_cluster:
                continue
            tj, zj = t[m], z[m]
            if np.ptp(tj) > 0:
                drifts.append(np.polyfit(tj, zj, 1)[0])
            if best is None or zj.mean() < best[0]:
                best = (zj.mean(), tj.mean())
        if best is None or not drifts:
            return None
        drift = float(np.mean(drifts))
        off = best[0] + drift * (tq - best[1]) - c.bias_us
        return dict(offset_us=off, drift_ppm=drift, sigma_us=math.nan, mode="event")


class HybridKF:
    """Our method: one Kalman filter for packets and events.

    State x = [theta, alpha, b_pkt, b_g1, b_g2, ...]
      theta  offset of the reference event group, us
      alpha  drift, us/s (= ppm)
      b_pkt  constant bias of the packet measurement (difference of the two BLE
             minimum delays), us
      b_g    bias of event group g relative to the reference group, us.
             The reference group has b = 0 and zero variance.

    Measurements
      packet bin:  z = theta + b_pkt         (every kf_bin_s, if use_packets)
      event pair:  z = theta + b_g           (g = the group the event matches)

    An event that matches no group waits in `pending`. When group_min pending
    events agree within group_eps_us (after the drift is removed), they form a
    new group. If a new group is lower than the reference, it becomes the new
    reference (a closer source has less propagation delay).
    """

    def __init__(self, cfg, use_packets=True, use_events=True):
        self.cfg = cfg
        self.use_packets = use_packets
        self.use_events = use_events
        self.x = None
        self.P = None
        self.t = None
        self.ref = None
        self.groups = []
        self.counts = {}
        self.pending = []
        self.updates = 0

    # -- Kalman basics ----------------------------------------------------
    def _advance(self, t):
        """Predict forward to t. If t is in the past, return the lag (s)."""
        if t <= self.t:
            return self.t - t
        c = self.cfg
        dt = t - self.t
        n = len(self.x)
        F = np.eye(n)
        F[0, 1] = dt
        Q = np.zeros((n, n))
        Q[0, 0] = c.q_theta * dt + c.q_alpha * dt ** 3 / 3
        Q[0, 1] = Q[1, 0] = c.q_alpha * dt * dt / 2
        Q[1, 1] = c.q_alpha * dt
        Q[2, 2] = c.q_pkt * dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t = t
        return 0.0

    def _h(self, lag, col=None):
        """Row of H for theta at (self.t - lag), plus one bias column."""
        h = np.zeros(len(self.x))
        h[0] = 1.0
        h[1] = -lag
        if col is not None:
            h[col] += 1.0
        return h

    def _update(self, h, z, R):
        Ph = self.P @ h
        S = h @ Ph + R
        K = Ph / S
        self.x = self.x + K * (z - h @ self.x)
        A = np.eye(len(self.x)) - np.outer(K, h)
        self.P = A @ self.P @ A.T + np.outer(K, K) * R  # Joseph form
        self.updates += 1

    def _add_state(self, value, var, cov):
        n = len(self.x)
        self.x = np.append(self.x, value)
        P = np.zeros((n + 1, n + 1))
        P[:n, :n] = self.P
        P[n, :n] = P[:n, n] = cov
        P[n, n] = var
        self.P = P
        return n

    # -- measurements -----------------------------------------------------
    def packet(self, t, z):
        if not self.use_packets:
            return
        c = self.cfg
        if self.x is None:
            # theta = z - b_pkt, with b_pkt ~ N(0, bpkt0^2)
            bp2, s2 = c.bpkt0_us ** 2, c.sigma_pkt_us ** 2
            self.x = np.array([float(z), 0.0, 0.0])
            self.P = np.array([[bp2 + s2, 0, -bp2], [0, c.alpha0_ppm ** 2, 0], [-bp2, 0, bp2]])
            self.t = t
            return
        lag = self._advance(t)
        h = self._h(lag)
        h[2] = 1.0
        self._update(h, z, c.sigma_pkt_us ** 2)

    def event(self, t, z, sigma):
        if not self.use_events:
            return "off"
        R = sigma ** 2
        if self.x is not None and self.groups:
            lag = self._advance(t)
            best = None
            for col in self.groups:
                h = self._h(lag, col)
                S = h @ self.P @ h + R
                nis = (z - h @ self.x) ** 2 / S
                if best is None or nis < best[0]:
                    best = (nis, col, h)
            if best[0] < self.cfg.gate ** 2:
                self._update(best[2], z, R)
                self.counts[best[1]] += 1
                return "matched"
        self.pending.append((t, float(z), R))
        return "grouped" if self._try_form_group() else "pending"

    def _try_form_group(self):
        c = self.cfg
        tnow = max(p[0] for p in self.pending)
        self.pending = [p for p in self.pending if p[0] >= tnow - c.pending_timeout_s]
        if len(self.pending) < c.group_min:
            return False
        if self.use_packets and self.x is None:
            return False  # hybrid: wait for the first packet measurement
        ts = np.array([p[0] for p in self.pending])
        zs = np.array([p[1] for p in self.pending])
        if self.x is None:
            # Events-only start: drift unknown. Find the line (offset + drift) that
            # most events agree with. Ask for one more event than usual.
            lo, hi, need = -c.max_drift_ppm, c.max_drift_ppm, c.group_min + 1
        else:
            # Drift known to +/- 3 sigma (from packets in hybrid mode).
            sd = 3 * math.sqrt(max(self.P[1, 1], 0.0))
            lo, hi, need = self.x[1] - sd, self.x[1] + sd, c.group_min
        members, slope = ransac_line(ts, zs, c.group_eps_us, lo, hi)
        if len(members) < need:
            return False
        members = sorted(members, key=lambda i: ts[i])
        chosen = [self.pending[i] for i in members]
        drop = {id(p) for p in chosen}
        self.pending = [p for p in self.pending if id(p) not in drop]

        if self.x is None:
            # Events-only filter: start the state from the first member.
            t0, z0, R0 = chosen[0]
            self.x = np.array([z0, slope, 0.0])
            self.P = np.diag([R0, c.alpha0_ppm ** 2, c.bpkt0_us ** 2])
            self.t = t0
            col = self._add_state(0.0, 0.0, 0.0)
            self.ref = col
            rest = chosen[1:]
        elif self.ref is None:
            # First group: it becomes the reference (bias fixed at 0).
            col = self._add_state(0.0, 0.0, 0.0)
            self.ref = col
            rest = chosen
        else:
            # New group: its bias starts from the first member (exact augmentation).
            t0, z0, R0 = chosen[0]
            h = self._h(self._advance(t0))
            Ph = self.P @ h
            col = self._add_state(z0 - h @ self.x, h @ Ph + R0, -Ph)
            rest = chosen[1:]
        self.groups.append(col)
        self.counts[col] = len(chosen)
        for t, z, R in rest:
            self._update(self._h(self._advance(t), col), z, R)
        if col != self.ref:
            b, sd = self.x[col], math.sqrt(max(self.P[col, col], 0.0))
            if b < -max(c.reref_margin_us, 3 * sd):
                self._rereference(col)
        return True

    def _rereference(self, col):
        """Make group `col` the reference: theta += b_col, all other biases -= b_col."""
        n = len(self.x)
        T = np.eye(n)
        T[0, col] += 1.0
        T[2, col] -= 1.0
        for g in self.groups:
            T[g, col] -= 1.0
        self.x = T @ self.x
        self.P = T @ self.P @ T.T
        self.x[col] = 0.0
        self.P[col, :] = 0.0
        self.P[:, col] = 0.0
        self.ref = col

    def estimate(self, t):
        if self.x is None:
            return None
        dt = t - self.t
        theta = self.x[0] + self.x[1] * dt
        var = self.P[0, 0] + 2 * dt * self.P[0, 1] + dt * dt * self.P[1, 1]
        mode = "event" if self.ref is not None else "packet"
        off = theta - (self.cfg.bias_us if mode == "event" else 0.0)
        return dict(offset_us=off, drift_ppm=float(self.x[1]),
                    sigma_us=math.sqrt(max(var, 0.0)), mode=mode)


def make_estimators(cfg, methods=METHODS):
    table = {
        "haest": lambda: HaestBaseline(cfg),
        "packets": lambda: HybridKF(cfg, use_packets=True, use_events=False),
        "ours_events": lambda: HybridKF(cfg, use_packets=False, use_events=True),
        "ours_hybrid": lambda: HybridKF(cfg, use_packets=True, use_events=True),
    }
    return {m: table[m]() for m in methods}


# --------------------------------------------------------------------------
# Online pipeline: feed packets in arrival order.
# --------------------------------------------------------------------------

class OnlineSync:
    def __init__(self, cfg, methods=METHODS, auto_pair=True):
        self.cfg = cfg
        self.auto_pair = auto_pair
        self.t0_us = None
        self.now = 0.0
        self.coarse = {n: Envelope(cfg.coarse_bin_s) for n in NODES}
        self.kfbin = {n: Envelope(cfg.kf_bin_s) for n in NODES}
        self.bins = {n: {} for n in NODES}
        self.det = {n: EventDetector(cfg.holdoff_ms) for n in NODES}
        self.levels = {n: deque(maxlen=200) for n in NODES}
        self.pairer = Pairer(cfg)
        self.events = {n: [] for n in NODES}
        self.pairs = []        # (t_pi, z, sigma)
        self.packet_meas = []  # (t, z)
        self.est = make_estimators(cfg, methods)
        self.n_packets = 0
        self.truth_open = {n: [] for n in NODES}  # unmatched pulses: (t_pi, t_edge)
        self.truth_pairs = []                      # (t_pi, t_acu, t_sound)

    def on_packet(self, p):
        if self.t0_us is None:
            self.t0_us = p.pi_us
        t = (p.pi_us - self.t0_us) / 1e6
        self.now = max(self.now, t)
        self.n_packets += 1
        u = p.tx - p.pi_us
        self.coarse[p.node].add(t, u)
        done = self.kfbin[p.node].add(t, u)
        if done is not None:
            self._bin_done(p.node, *done)
        if p.trigger == 2:  # truth pulse: evaluation only, never an event
            if p.t_edge > 0:
                t_pi = (p.t_edge - self.coarse[p.node].value() - self.t0_us) / 1e6
                self._truth_pulse(p.node, t_pi, p.t_edge)
            return
        if p.trigger == 1 and p.t_edge > 0 and self.det[p.node].is_new(p.t_edge):
            t_pi = (p.t_edge - self.coarse[p.node].value() - self.t0_us) / 1e6
            ev = Event(p.node, t_pi, p.t_edge, p.level)
            self.events[p.node].append(ev)
            self.levels[p.node].append(p.level)
            if self.auto_pair:
                self.pairer.add(ev)
        if self.auto_pair:
            for a, s in self.pairer.resolve(self.now):
                self.add_pair(a, s)

    def _truth_pulse(self, node, t_pi, t_edge):
        """Pair a truth pulse with the other node's pulse of the same instant."""
        other = NODE_SOUND if node == NODE_ACU else NODE_ACU
        cands = [x for x in self.truth_open[other] if abs(x[0] - t_pi) <= self.cfg.truth_match_s]
        if len(cands) == 1:
            self.truth_open[other].remove(cands[0])
            if node == NODE_ACU:
                self.truth_pairs.append((t_pi, t_edge, cands[0][1]))
            else:
                self.truth_pairs.append((cands[0][0], cands[0][1], t_edge))
        else:
            self.truth_open[node].append((t_pi, t_edge))
        keep = self.now - 10.0
        for n in NODES:
            self.truth_open[n] = [x for x in self.truth_open[n] if x[0] >= keep]

    def _bin_done(self, node, k, umax, n):
        c = self.cfg
        if n < c.kf_bin_min_frac * c.kf_bin_s / 0.05:
            return
        self.bins[node][k] = umax
        other = self.bins[NODE_ACU if node == NODE_SOUND else NODE_SOUND]
        if k in other:
            z = self.bins[NODE_SOUND][k] - self.bins[NODE_ACU][k]
            tk = (k + 0.5) * c.kf_bin_s
            self.packet_meas.append((tk, z))
            for e in self.est.values():
                e.packet(tk, z)
            for n_ in NODES:
                self.bins[n_] = {kk: v for kk, v in self.bins[n_].items() if kk > k - 5}
                self.bins[n_].pop(k, None)

    def _weight(self, ev):
        lv = self.levels[ev.node]
        med = float(np.median(lv)) if len(lv) >= 5 else max(ev.level, 1)
        return min(3.0, max(0.5, math.sqrt(max(med, 1.0) / max(ev.level, 1.0))))

    def add_pair(self, a, s):
        c = self.cfg
        z = s.t_edge - a.t_edge
        sigma = c.sigma_ev_us
        if c.use_weights:
            wa, ws = self._weight(a), self._weight(s)
            sigma *= math.sqrt((wa * wa + ws * ws) / 2)
        self.pairs.append((a.t_pi, float(z), sigma))
        for e in self.est.values():
            e.event(a.t_pi, z, sigma)

    def estimate(self, t):
        return {m: e.estimate(t) for m, e in self.est.items()}


# --------------------------------------------------------------------------
# Pattern pairing: pair events by their time pattern, with no packet timing.
# --------------------------------------------------------------------------

def pattern_pairs(acu, snd, cfg):
    """Pair events by the gaps between them.

    In each window, find the offset h that lines up the most ACU events with
    sound events (the peak of all differences s - a). Then pair each ACU event
    with the one sound event near a + h. Taps must not be perfectly periodic.
    """
    if len(acu) < 3 or len(snd) < 3:
        return []
    A = np.array([e.t_edge for e in acu], dtype=np.int64)
    S = np.array([e.t_edge for e in snd], dtype=np.int64)
    tol = cfg.pattern_tol_us
    win = cfg.pattern_window_s * 1e6
    out, h = [], None
    start = A[0]
    while start <= A[-1]:
        ia = np.nonzero((A >= start) & (A < start + win))[0]
        start += win
        if len(ia) < 3:
            continue
        d = (S[None, :] - A[ia][:, None]).ravel().astype(float)
        if h is not None:
            d = d[np.abs(d - h) < 1e6]
        if len(d) < 3:
            continue
        band = densest_cluster(d, tol)
        if len(band) < 3:
            continue
        h_win = float(np.median(d[band]))
        used = {}
        for i in ia:
            target = A[i] + h_win
            j = int(np.searchsorted(S, target))
            cand = [jj for jj in (j - 1, j) if 0 <= jj < len(S) and abs(S[jj] - target) <= tol]
            if len(cand) == 1:
                used.setdefault(cand[0], []).append(i)
        got = [(acu[v[0]], snd[k]) for k, v in used.items() if len(v) == 1]
        if got:
            h = float(np.median([s.t_edge - a.t_edge for a, s in got]))
            out.extend(got)
    out.sort(key=lambda p: p[0].t_pi)
    return out


# --------------------------------------------------------------------------
# Offline replay, evaluation, plots
# --------------------------------------------------------------------------

def truth_to_pi(rows, truth, t0_us):
    """Map each truth instant to Pi time (s) with the ACU packet envelope."""
    acu = [p for p in rows if p.node == NODE_ACU]
    tx = np.array([p.tx for p in acu], dtype=np.int64)
    u = np.array([p.tx - p.pi_us for p in acu])
    out = []
    for ta, ts in truth:
        i = int(np.searchsorted(tx, ta))
        if i <= 0 or i >= len(tx):
            continue
        umax = u[max(0, i - 40):i + 40].max()
        out.append(((ta - umax - t0_us) / 1e6, float(ts - ta)))
    return out


def truth_from_csv(rows, cfg):
    """Truth pairs (t_acu_us, t_sound_us) from the truth-pulse packets (trigger = 2)."""
    s = OnlineSync(cfg, methods=(), auto_pair=False)
    for p in rows:
        s.on_packet(p)
    return [(ta, ts) for _t, ta, ts in s.truth_pairs]


def replay(rows, cfg, truth=None, pairing="coarse", grid_s=5.0, methods=METHODS):
    """Run the online pipeline over a whole CSV. Estimates are causal: each one
    uses only packets that arrived before its query time."""
    t0 = rows[0].pi_us
    t_end = (rows[-1].pi_us - t0) / 1e6
    queries = [(t, None) for t in np.arange(grid_s, t_end, grid_s)]
    if truth:
        queries += truth_to_pi(rows, truth, t0)
    queries.sort(key=lambda q: q[0])

    inject = []
    if pairing == "pattern":
        first = OnlineSync(cfg, methods=(), auto_pair=False)
        for p in rows:
            first.on_packet(p)
        inject = pattern_pairs(first.events[NODE_ACU], first.events[NODE_SOUND], cfg)
    delay = cfg.pair_window_ms / 1000.0 + cfg.resolve_margin_s

    sync = OnlineSync(cfg, methods=methods, auto_pair=(pairing == "coarse"))
    records, qi, ii = [], 0, 0
    cpu0 = time.process_time()
    for p in rows:
        t = (p.pi_us - t0) / 1e6
        while qi < len(queries) and queries[qi][0] <= t:
            tq, tru = queries[qi]
            records.append((tq, tru, sync.estimate(tq)))
            qi += 1
        sync.on_packet(p)
        while ii < len(inject) and inject[ii][0].t_pi + delay <= sync.now:
            sync.add_pair(*inject[ii])
            ii += 1
    cpu = time.process_time() - cpu0
    sync.cpu_s = cpu
    sync.run_s = t_end
    sync.n_pattern_pairs = len(inject)
    return sync, records


def evaluate(records, methods=METHODS):
    """Error statistics against truth, per method.

    lock_s  first time the method gives an event-based estimate (HAEST: any estimate)
    mae_locked, p95_locked, max_locked  errors from lock_s on
    """
    tr = [r for r in records if r[1] is not None]
    res = {}
    for m in methods:
        got = [(r[0], r[2][m]["offset_us"] - r[1], r[2][m]["mode"]) for r in tr
               if r[2].get(m) is not None]
        e = np.abs(np.array([g[1] for g in got])) if got else np.array([])
        lock = next((g[0] for g in got if g[2] == "event"), math.nan)
        el = np.abs(np.array([g[1] for g in got if g[0] >= lock]))
        res[m] = dict(n=len(tr), coverage=len(got) / max(len(tr), 1),
                      mae=float(e.mean()) if len(e) else math.nan,
                      rms=float(np.sqrt((e ** 2).mean())) if len(e) else math.nan,
                      lock_s=lock,
                      mae_locked=float(el.mean()) if len(el) else math.nan,
                      p95_locked=float(np.percentile(el, 95)) if len(el) else math.nan,
                      max_locked=float(el.max()) if len(el) else math.nan)
    return res


def summary(sync, records, methods=METHODS, out=None):
    out = out or sys.stdout
    pr = sync.pairer
    nA, nS = len(sync.events[NODE_ACU]), len(sync.events[NODE_SOUND])
    print(f"Run: {sync.run_s / 60:.1f} min, {sync.n_packets} packets, "
          f"{nA} ACU events, {nS} sound events", file=out)
    if sync.auto_pair:
        print(f"Pairs: {len(sync.pairs)} kept, {pr.ambiguous} ambiguous (dropped), "
              f"{pr.unmatched} ACU events with no sound event", file=out)
    else:
        print(f"Pairs (pattern): {sync.n_pattern_pairs}", file=out)
    print(f"CPU: {sync.cpu_s:.2f} s for the whole run "
          f"({100 * sync.cpu_s / max(sync.run_s, 1e-9):.3f} % of run time, all methods)", file=out)
    last = records[-1][2] if records else {}
    kf = sync.est.get("ours_hybrid")
    if kf is not None:
        print(f"Hybrid: {len(kf.groups)} event group(s), {kf.updates} filter updates, "
              f"{len(kf.pending)} pending", file=out)
    has_truth = any(r[1] is not None for r in records)
    res = evaluate(records, methods) if has_truth else {}
    print(file=out)
    if has_truth:
        print(f"{'':12} {'':>6} {'whole':>8} {'lock':>7} {'------ after lock ------':>26}", file=out)
        print(f"{'method':12} {'cover':>6} {'MAE us':>8} {'s':>7} {'MAE us':>8} {'p95 us':>8} "
              f"{'max us':>8} {'drift ppm':>10}", file=out)
    else:
        print(f"{'method':12} {'offset us':>14} {'drift ppm':>10} {'sigma us':>9} mode", file=out)
    for m in methods:
        e = last.get(m)
        drift = f"{e['drift_ppm']:10.2f}" if e else f"{'-':>10}"
        if has_truth:
            r = res[m]
            print(f"{m:12} {100 * r['coverage']:5.0f}% {r['mae']:8.0f} {r['lock_s']:7.0f} "
                  f"{r['mae_locked']:8.0f} {r['p95_locked']:8.0f} {r['max_locked']:8.0f} {drift}",
                  file=out)
        elif e:
            print(f"{m:12} {e['offset_us']:14.0f} {drift} {e['sigma_us']:9.0f} {e['mode']}",
                  file=out)
        else:
            print(f"{m:12} {'no estimate':>14}", file=out)
    return res


def write_estimates(path, records, methods=METHODS):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        head = ["t_s", "truth_offset_us"]
        for m in methods:
            head += [f"{m}_offset_us", f"{m}_drift_ppm", f"{m}_sigma_us", f"{m}_mode"]
        w.writerow(head)
        for t, tru, est in records:
            row = [f"{t:.3f}", "" if tru is None else f"{tru:.1f}"]
            for m in methods:
                e = est.get(m)
                row += ([f"{e['offset_us']:.1f}", f"{e['drift_ppm']:.3f}",
                         f"{e['sigma_us']:.1f}", e["mode"]] if e else ["", "", "", ""])
            w.writerow(row)


def plot(path, rows, sync, records, methods=METHODS):
    """Deliverable 3: delay, offset and drift over time (and error if there is truth)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t0 = rows[0].pi_us
    has_truth = any(r[1] is not None for r in records)
    fig, ax = plt.subplots(4 if has_truth else 3, 1, figsize=(10, 12 if has_truth else 9),
                           sharex=True)
    colors = {"haest": "#d1495b", "packets": "#8d99ae", "ours_events": "#edae49",
              "ours_hybrid": "#00798c"}

    # 1. BLE delay above the minimum, per node
    top99 = 0.0
    for n, name, col in ((NODE_ACU, "ACU", "#30638e"), (NODE_SOUND, "sound", "#d1495b")):
        ps = [p for p in rows if p.node == n]
        t = np.array([(p.pi_us - t0) / 1e6 for p in ps])
        u = np.array([p.tx - p.pi_us for p in ps])
        k = (t // sync.cfg.coarse_bin_s).astype(int)
        top = {}
        for kk, uu in zip(k, u):
            top[kk] = max(top.get(kk, -math.inf), uu)
        excess = np.array([top[kk] - uu for kk, uu in zip(k, u)])
        ax[0].plot(t / 60, excess / 1000, ".", ms=1, color=col, label=name, alpha=0.5)
        top99 = max(top99, np.percentile(excess, 99.5) / 1000) if len(excess) else top99
    ax[0].set_ylim(-1, top99 * 1.1 + 1)
    ax[0].set_ylabel("BLE delay above\nminimum (ms)")
    ax[0].legend(loc="upper right", markerscale=8)

    # 2. Offset. Subtract one constant so ms-size changes are visible.
    ref = next((r[2]["ours_hybrid"]["offset_us"] for r in records
                if r[2].get("ours_hybrid") and r[2]["ours_hybrid"]["mode"] == "event"), None)
    if ref is None:
        ref = next((e["offset_us"] for r in records for e in r[2].values() if e), 0.0)
    C = round(ref / 1000.0) * 1000.0
    if sync.pairs:
        pt = np.array([p[0] for p in sync.pairs])
        pz = np.array([p[1] for p in sync.pairs]) - sync.cfg.bias_us
        ax[1].plot(pt / 60, (pz - C) / 1000, ".", ms=2, color="#bbbbbb", label="event pairs")
    for m in methods:
        pts = [(r[0], r[2][m]["offset_us"]) for r in records if r[2].get(m)]
        if pts:
            a = np.array(pts)
            ax[1].plot(a[:, 0] / 60, (a[:, 1] - C) / 1000, "-", color=colors[m], label=m, lw=1.2)
    if has_truth:
        a = np.array([(r[0], r[1]) for r in records if r[1] is not None])
        ax[1].plot(a[:, 0] / 60, (a[:, 1] - C) / 1000, "k+", ms=4, label="truth")
    ax[1].set_ylabel(f"offset - {C / 1e6:.3f} s\n(ms)")
    ax[1].legend(loc="upper right", fontsize=8, ncol=3)

    # 3. Drift
    for m in methods:
        pts = [(r[0], r[2][m]["drift_ppm"]) for r in records if r[2].get(m)]
        if pts:
            a = np.array(pts)
            ax[2].plot(a[:, 0] / 60, a[:, 1], "-", color=colors[m], label=m, lw=1.2)
    ax[2].set_ylabel("drift (ppm)")
    ax[2].legend(loc="upper right", fontsize=8, ncol=2)

    # 4. Error vs truth
    if has_truth:
        for m in methods:
            pts = [(r[0], r[2][m]["offset_us"] - r[1]) for r in records
                   if r[1] is not None and r[2].get(m)]
            if pts:
                a = np.array(pts)
                ax[3].plot(a[:, 0] / 60, a[:, 1] / 1000, "-", color=colors[m], label=m, lw=1)
        ax[3].axhspan(-1, 1, color="#00798c", alpha=0.08)
        ax[3].set_ylim(-5, 5)
        ax[3].set_ylabel("error (ms)\nshaded = 1 ms goal")
    ax[-1].set_xlabel("time (min)")
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=130)


# --------------------------------------------------------------------------
# Live mode
# --------------------------------------------------------------------------

def follow(path, cfg, every_s=5.0):
    """Read the CSV as ble_rx.py appends to it, and print the hybrid estimate."""
    sync = OnlineSync(cfg, methods=("ours_hybrid",))
    cpu0, wall0 = time.process_time(), time.monotonic()
    last = wall0
    buf = ""
    errs, n_truth = [], 0  # live error against truth pulses (event mode only)
    with open(path, newline="") as f:
        while True:
            chunk = f.readline()
            if chunk:
                buf += chunk
                if not buf.endswith("\n"):
                    continue
                p = parse_row(buf.strip().split(","))
                buf = ""
                if p is not None:
                    sync.on_packet(p)
                    while n_truth < len(sync.truth_pairs):
                        t_pi, ta, ts = sync.truth_pairs[n_truth]
                        n_truth += 1
                        e = sync.estimate(t_pi)["ours_hybrid"]
                        if e and e["mode"] == "event":
                            errs.append(e["offset_us"] - (ts - ta))
                continue
            now = time.monotonic()
            if now - last >= every_s and sync.t0_us is not None:
                last = now
                e = sync.estimate(sync.now)["ours_hybrid"]
                cpu = 100 * (time.process_time() - cpu0) / max(now - wall0, 1e-9)
                kf = sync.est["ours_hybrid"]
                if e:
                    print(f"t={sync.now:8.1f}s offset={e['offset_us']:14.0f} us "
                          f"+/-{e['sigma_us']:6.0f}  drift={e['drift_ppm']:7.2f} ppm  "
                          f"mode={e['mode']:6} groups={len(kf.groups)} pairs={len(sync.pairs)} "
                          f"cpu={cpu:.2f}%" +
                          (f"  truth: last err={errs[-1]:+.0f} us, MAE={np.mean(np.abs(errs)):.0f} us "
                           f"(n={len(errs)})" if errs else ""), flush=True)
                else:
                    print(f"t={sync.now:8.1f}s waiting for data, cpu={cpu:.2f}%", flush=True)
            time.sleep(0.2)


def config_from_args(args):
    cfg = Config()
    for f in fields(Config):
        v = getattr(args, f.name, None)
        if v is not None:
            setattr(cfg, f.name, v)
    return cfg


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", help="CSV from ble_rx.py (or sync_sim.py)")
    p.add_argument("--truth", help="truth CSV: t_acu_us,t_sound_us per row")
    p.add_argument("--pairing", choices=("coarse", "pattern"), default="coarse")
    p.add_argument("--plot", help="save plots to this PNG")
    p.add_argument("--out", help="save estimates to this CSV")
    p.add_argument("--follow", action="store_true", help="live mode")
    p.add_argument("--no-weights", dest="use_weights", action="store_false", default=None)
    for f in fields(Config):
        if f.name == "use_weights":
            continue
        p.add_argument("--" + f.name.replace("_", "-"), dest=f.name, type=type(f.default))
    args = p.parse_args(argv)
    cfg = config_from_args(args)

    if args.follow:
        try:
            follow(args.csv, cfg)
        except KeyboardInterrupt:
            pass
        return 0

    rows = load_csv(args.csv)
    if len(rows) < 10:
        print("not enough packets in the CSV", file=sys.stderr)
        return 1
    if args.truth:
        truth = load_truth(args.truth)
    else:
        truth = truth_from_csv(rows, cfg) or None
        if truth:
            print(f"Truth: {len(truth)} truth-pulse pairs from the CSV")
    sync, records = replay(rows, cfg, truth=truth, pairing=args.pairing)
    summary(sync, records)
    if args.out:
        write_estimates(args.out, records)
    if args.plot:
        plot(args.plot, rows, sync, records)
    return 0


if __name__ == "__main__":
    sys.exit(main())
