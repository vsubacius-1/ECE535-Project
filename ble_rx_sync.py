#!/usr/bin/env python3
"""Receive sensor packets from both ESP32 nodes over BLE, log them, and synchronize their clocks.

Nodes (both use the same service/characteristic UUIDs and the same 33-byte packet):
  TSN-Sound  node 2  KY-037 microphone
  TSN-ACU    node 1  ADXL345 accelerometer

Install:  pip install bleak            (setup_ble.sh does this in ~/tsn-venv)
Run:      python3 ble_rx_sync.py                         # both nodes, print to terminal
          python3 ble_rx_sync.py --out run.csv           # save raw packets + run.sync.csv
          python3 ble_rx_sync.py --names TSN-ACU         # only one node (no cross-node sync)
          python3 ble_rx_sync.py --replay run.csv        # re-run the sync on a recorded file

Raw CSV columns (unchanged from ble_rx.py, so old recordings can be replayed):
  pi_rx_ns   Pi CLOCK_MONOTONIC_RAW when the packet arrived (ns); not slewed by NTP
  node       1 = ACU (ADXL345), 2 = sound (KY-037)
  seq        packet counter from that ESP32 (wraps at 65535)
  t_edge_us  ESP32 clock at the first event edge in the 50 ms window, 0 = none
  t_peak_us  ESP32 clock at the largest sample in the window
  tx_us      ESP32 clock just before the packet was sent
  raw        sound: last AO reading (0-4095);  ACU: |a| in mg
  level      sound: noise-subtracted level;    ACU: peak-to-peak |a| in mg
  trigger    1 if t_edge_us is set

Synchronization runs in two stages, using only the packets the nodes already send:

1. Node -> Pi clock map (per node, from tx_us of every packet).
   d = pi_rx - tx_us = (Pi clock - node clock) + network delay. The network delay is never
   below some floor, so the smallest d in each few-second bin is a packet that got through
   with roughly the minimum delay. A line through those minima over time gives
   offset (intercept) and drift (slope) of the node against the Pi.
   d minus that line is the packet's delay above the floor: the network jitter (Deliverable 1).

2. Node <-> node sync from shared physical events (t_edge_us).
   A knock is heard by the KY-037 and felt by the ADXL345 at nearly the same instant.
   Stage 1 maps each edge to Pi time accurately enough (a few ms) to pair the detections;
   each pair (t_ref, t_other) is then an exact reading of the two ESP32 clocks at the same
   moment, with no BLE delay in it. A robust line through the pairs gives
   offset and relative drift between the two nodes (Deliverable 2).

Sync CSV columns (one row per estimate; unused columns are blank):
  kind=clock  node vs Pi:  offset_us (Pi - node, includes the minimum delay), skew_ppm
              (>0: Pi clock runs faster), n bins, spread_us (floor noise),
              delay_p50/p95/max_us = delay above the floor since the last report
  kind=pair   a matched event: t_ref_us, t_node_us, residual_us against the current fit
  kind=sync   node vs reference node: offset_us = node clock - ref clock at t_ref_us,
              skew_ppm (>0: node clock runs faster), n pairs, spread_us,
              ble_offset_us = the same offset estimated from stage 1 alone (for comparison)
  To convert a reference-node time t to the other node's clock: t + offset_us + skew_ppm*1e-6*(t - t_ref_us)
"""

import argparse
import asyncio
import os
import statistics
import struct
import sys
import time
from collections import deque

DEFAULT_NAMES = ["TSN-Sound", "TSN-ACU"]
NODE_LABELS = {1: "ACU", 2: "Sound"}
DATA_CHAR_UUID = "8ef8eb63-4406-4d27-b44e-b41ce16723f2"

PACKET_FMT = "<BBHQQQHHB"  # little-endian, no padding
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # 33
PACKET_VERSION = 1
HEADER = "pi_rx_ns,node,seq,t_edge_us,t_peak_us,tx_us,raw,level,trigger"
SYNC_COLS = ["pi_rx_ns", "kind", "node", "offset_us", "skew_ppm", "n", "spread_us", "ble_offset_us",
             "delay_p50_us", "delay_p95_us", "delay_max_us", "t_ref_us", "t_node_us", "residual_us"]
SCAN_TIMEOUT_S = 5.0

# CLOCK_MONOTONIC is frequency-adjusted by NTP, which would show up as fake drift. RAW is not.
if hasattr(time, "clock_gettime_ns") and hasattr(time, "CLOCK_MONOTONIC_RAW"):
    def now_ns():
        return time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
else:
    now_ns = time.monotonic_ns


def log_msg(msg):
    print(f"# {msg}", file=sys.stderr, flush=True)


def label(node):
    return NODE_LABELS.get(node, f"node{node}")


def decode(data: bytes):
    """Return the packet fields (without version) as a tuple, or None if not valid."""
    if len(data) != PACKET_SIZE:
        return None
    fields = struct.unpack(PACKET_FMT, data)
    if fields[0] != PACKET_VERSION:
        return None
    return fields[1:]


# ---------------------------------------------------------------------------------------------
# Fitting helpers (pure Python: the point sets are small, and this keeps the Pi install light)
# ---------------------------------------------------------------------------------------------

def _lstsq(xs, ys, idx, slope):
    n = len(idx)
    if slope is not None:
        return sum(ys[i] - slope * xs[i] for i in idx) / n, slope
    mx = sum(xs[i] for i in idx) / n
    my = sum(ys[i] for i in idx) / n
    sxx = sum((xs[i] - mx) ** 2 for i in idx)
    if sxx == 0:
        return my, 0.0
    b = sum((xs[i] - mx) * (ys[i] - my) for i in idx) / sxx
    return my - b * mx, b


def robust_line(xs, ys, slope=None, lower=False, k=3.0, min_tol=0.0, iters=4):
    """Fit y = a + b*x by least squares, dropping outliers and refitting.

    slope:  fix b and fit only a.
    lower:  fit the lower envelope; only points far ABOVE the line are dropped
            (used for minimum delays, where noise is only ever positive).
    Returns (a, b, spread, n_used); spread is the robust std of the residuals.
    """
    idx = list(range(len(xs)))
    a, b = _lstsq(xs, ys, idx, slope)
    spread = 0.0
    for _ in range(iters):
        res = [ys[i] - (a + b * xs[i]) for i in idx]
        spread = 1.4826 * statistics.median(abs(r) for r in res)
        tol = max(k * spread, min_tol)
        keep = [i for i, r in zip(idx, res) if (r <= tol if lower else abs(r) <= tol)]
        if len(keep) == len(idx) or len(keep) < 2:
            break
        idx = keep
        a, b = _lstsq(xs, ys, idx, slope)
    return a, b, spread, len(idx)


def lower_hull_line(xs, ys):
    """Line under all points that hugs them as closely as possible (minimizes the summed gap).

    Standard one-way-delay skew estimator: delay noise is only ever positive, so the true
    floor is a line every point lies on or above. The optimum is the lower-convex-hull edge
    that spans the mean of x. xs must be increasing. Returns (a, b).
    """
    hull = []
    for p in zip(xs, ys):
        while len(hull) >= 2:
            (x1, y1), (x2, y2) = hull[-2], hull[-1]
            if (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1) <= 0:
                hull.pop()
            else:
                break
        hull.append(p)
    if len(hull) == 1:
        return hull[0][1], 0.0
    xm = sum(xs) / len(xs)
    for (x1, y1), (x2, y2) in zip(hull, hull[1:]):
        if x2 >= xm:
            b = (y2 - y1) / (x2 - x1)
            return y1 - b * x1, b
    (x1, y1), (x2, y2) = hull[-2], hull[-1]
    b = (y2 - y1) / (x2 - x1)
    return y1 - b * x1, b


def percentile(sorted_xs, p):
    if not sorted_xs:
        return None
    return sorted_xs[min(len(sorted_xs) - 1, int(p * len(sorted_xs)))]


# ---------------------------------------------------------------------------------------------
# Stage 1: node clock -> Pi clock, from tx_us and arrival time of every packet
# ---------------------------------------------------------------------------------------------

class ClockMap:
    def __init__(self, bin_s, window_s):
        self.bin_us = int(bin_s * 1e6)
        self.max_bins = max(3, int(window_s / bin_s))
        self.reset()

    def reset(self):
        self.bins = deque()      # [bin index, tx_us, d_us] = smallest d seen in each bin
        self.x0 = None           # tx_us origin, keeps the floats well conditioned
        self.a = self.b = 0.0    # d(tx) = a + b * (tx - x0)
        self.spread = 0.0
        self.n = 0
        self.ready = False
        self.last_tx = None
        self.excess = []         # delay above the floor, since the last report

    def add(self, tx_us, rx_ns):
        if self.last_tx is not None and tx_us < self.last_tx - 1_000_000:
            log_msg("node clock went backwards (ESP32 reboot?), restarting its clock map")
            self.reset()
        self.last_tx = max(tx_us, self.last_tx or 0)
        d = rx_ns / 1000.0 - tx_us
        if self.x0 is None:
            self.x0 = tx_us
        k = tx_us // self.bin_us
        if self.bins and self.bins[-1][0] == k:
            if d < self.bins[-1][2]:
                self.bins[-1][1], self.bins[-1][2] = tx_us, d
        else:
            self.bins.append([k, tx_us, d])
            while len(self.bins) > self.max_bins:
                self.bins.popleft()
            self._refit()  # once per bin: cheap enough for the < 5 % CPU goal
        if self.ready:
            self.excess.append(d - self.model_d(tx_us))

    def _refit(self):
        done = list(self.bins)[:-1]  # the newest bin is still filling up
        if not done:
            return
        xs = [tx - self.x0 for _, tx, _ in done]
        ys = [d for _, _, d in done]
        if len(done) < 3:  # too short to see drift yet; offset only
            self.a, self.b, self.spread, self.n = min(ys), 0.0, 0.0, len(done)
        else:
            self.a, self.b = lower_hull_line(xs, ys)
            self.n = len(done)
            self.spread = statistics.median(y - (self.a + self.b * x) for x, y in zip(xs, ys))
        self.ready = True

    def model_d(self, t_us):
        return self.a + self.b * (t_us - self.x0)

    def to_pi_us(self, t_us):
        """Node time -> Pi time (us). Includes the minimum network delay as a constant."""
        return t_us + self.model_d(t_us)

    def from_pi_us(self, p_us):
        return (p_us - self.a + self.b * self.x0) / (1.0 + self.b)

    def take_excess(self):
        xs, self.excess = sorted(self.excess), []
        return xs


# ---------------------------------------------------------------------------------------------
# Stage 2: node <-> node, from the same physical event seen by both nodes
# ---------------------------------------------------------------------------------------------

class EventSync:
    def __init__(self, maps, ref_node, pair_window_ms, refractory_ms, min_pairs_skew,
                 min_span_s, max_pairs):
        self.maps = maps
        self.ref = ref_node
        self.window_us = pair_window_ms * 1000.0
        self.refractory_us = refractory_ms * 1000.0
        self.min_pairs_skew = min_pairs_skew
        self.min_span_us = min_span_s * 1e6
        self.max_pairs = max_pairs
        self.last_edge = {}   # node -> t_edge_us of the latest edge (for onset detection)
        self.pending = {}     # node -> deque of [pi_us, t_us, matched nodes] not yet paired
        self.pairs = {}       # other node -> deque of (t_ref_us, t_other_us)
        self.fits = {}        # other node -> dict(a, b, x0, spread, n, slope_from)

    def on_edge(self, node, t_us):
        """Feed one edge; returns rows to log (pairs and updated fits)."""
        prev = self.last_edge.get(node)
        self.last_edge[node] = t_us
        if prev is not None and 0 <= t_us - prev < self.refractory_us:
            return []  # same event still ringing in the next 50 ms window
        m = self.maps.get(node)
        if m is None or not m.ready:
            return []
        pi_us = m.to_pi_us(t_us)
        self._prune(pi_us)
        rows = []
        if node == self.ref:
            entry = [pi_us, t_us, set()]
            for other, q in self.pending.items():
                if other == self.ref:
                    continue
                hit = self._match(q, pi_us)
                if hit is not None:
                    q.remove(hit)
                    entry[2].add(other)
                    rows += self._add_pair(other, t_us, hit[1])
            self.pending.setdefault(node, deque()).append(entry)
        else:
            q = self.pending.get(self.ref, deque())
            hit = self._match(q, pi_us, skip=node)
            if hit is not None:
                hit[2].add(node)
                rows += self._add_pair(node, hit[1], t_us)
            else:
                self.pending.setdefault(node, deque()).append([pi_us, t_us, set()])
        return rows

    def _match(self, q, pi_us, skip=None):
        cands = [e for e in q if abs(e[0] - pi_us) <= self.window_us and skip not in e[2]]
        return cands[0] if len(cands) == 1 else None  # ambiguous or none: do not guess

    def _prune(self, pi_us):
        for q in self.pending.values():
            while q and q[0][0] < pi_us - 5e6:
                q.popleft()

    def _add_pair(self, other, t_ref, t_other):
        pairs = self.pairs.setdefault(other, deque(maxlen=self.max_pairs))
        pairs.append((t_ref, t_other))
        self._refit(other)
        f = self.fits[other]
        resid = (t_other - t_ref) - (f["a"] + f["b"] * (t_ref - f["x0"]))
        return [dict(kind="pair", node=other, t_ref_us=t_ref, t_node_us=t_other, residual_us=resid),
                self.sync_row(other, t_ref)]

    def ble_rel_skew(self, other):
        """Rate of other clock vs ref clock implied by the two Pi clock maps."""
        return (1.0 + self.maps[self.ref].b) / (1.0 + self.maps[other].b) - 1.0

    def ble_offset(self, other, t_ref):
        return self.maps[other].from_pi_us(self.maps[self.ref].to_pi_us(t_ref)) - t_ref

    def _refit(self, other):
        pairs = self.pairs[other]
        x0 = pairs[0][0]
        xs = [r - x0 for r, _ in pairs]
        ys = [o - r for r, o in pairs]
        if len(pairs) >= self.min_pairs_skew and xs[-1] - xs[0] >= self.min_span_us:
            a, b, spread, n = robust_line(xs, ys, min_tol=200.0)
            src = "events"
        else:  # not enough events to see drift yet: borrow it from the BLE clock maps
            a, b, spread, n = robust_line(xs, ys, slope=self.ble_rel_skew(other), min_tol=200.0)
            src = "ble"
        self.fits[other] = dict(a=a, b=b, x0=x0, spread=spread, n=n, slope_from=src)

    def sync_row(self, other, t_ref):
        f = self.fits[other]
        return dict(kind="sync", node=other, offset_us=f["a"] + f["b"] * (t_ref - f["x0"]),
                    skew_ppm=f["b"] * 1e6, n=f["n"], spread_us=f["spread"],
                    ble_offset_us=self.ble_offset(other, t_ref), t_ref_us=t_ref)


# ---------------------------------------------------------------------------------------------
# Glue: feeds packets to both stages, writes the sync CSV, prints periodic summaries
# ---------------------------------------------------------------------------------------------

class SyncEngine:
    def __init__(self, args, out):
        self.args = args
        self.out = out
        self.maps = {}
        self.events = EventSync(self.maps, args.ref_node, args.pair_window_ms, args.refractory_ms,
                                args.min_pairs_skew, args.min_span_s, args.max_pairs)
        self.report_ns = int(args.report_s * 1e9)
        self.next_report = None
        self.last_rx = 0
        self.write_line(",".join(SYNC_COLS), header=True)

    def write_line(self, line, header=False):
        if header and hasattr(self.out, "tell"):
            try:
                if self.out.tell() != 0:
                    return
            except OSError:
                pass
        self.out.write(line + "\n")

    def write(self, rx_ns, row):
        row = dict(row, pi_rx_ns=rx_ns)
        cells = []
        for c in SYNC_COLS:
            v = row.get(c)
            if v is None:
                cells.append("")
            elif isinstance(v, float):
                cells.append(f"{v:.3f}" if c == "skew_ppm" else f"{v:.1f}")
            else:
                cells.append(str(v))
        self.write_line(",".join(cells))

    def on_packet(self, rx_ns, fields):
        node, _seq, t_edge, _t_peak, tx, _raw, _level, trigger = fields
        self.last_rx = rx_ns
        m = self.maps.get(node)
        if m is None:
            m = self.maps[node] = ClockMap(self.args.bin_s, self.args.window_s)
        m.add(tx, rx_ns)
        if trigger and t_edge:
            for row in self.events.on_edge(node, t_edge):
                self.write(rx_ns, row)
        if self.next_report is None:
            self.next_report = rx_ns + self.report_ns
        elif rx_ns >= self.next_report:
            self.report(rx_ns)
            self.next_report += self.report_ns * max(1, (rx_ns - self.next_report) // self.report_ns + 1)

    def report(self, rx_ns=None, final=False):
        rx_ns = rx_ns or self.last_rx
        for node, m in sorted(self.maps.items()):
            if not m.ready:
                continue
            ex = m.take_excess()
            row = dict(kind="clock", node=node, offset_us=m.model_d(m.last_tx), skew_ppm=m.b * 1e6,
                       n=m.n, spread_us=m.spread, delay_p50_us=percentile(ex, 0.5),
                       delay_p95_us=percentile(ex, 0.95), delay_max_us=ex[-1] if ex else None)
            self.write(rx_ns, row)
            if ex:
                log_msg(f"{label(node)} vs Pi: drift {m.b * 1e6:+.2f} ppm, delay above floor "
                        f"p50 {row['delay_p50_us'] / 1000:.2f} ms, p95 {row['delay_p95_us'] / 1000:.2f} ms")
        ref = self.maps.get(self.args.ref_node)
        for other, f in sorted(self.events.fits.items()):
            if ref is None or ref.last_tx is None or not ref.ready:
                continue
            row = self.events.sync_row(other, ref.last_tx)
            self.write(rx_ns, row)
            log_msg(f"{label(other)} - {label(self.args.ref_node)}: offset {row['offset_us'] / 1000:+.3f} ms, "
                    f"drift {row['skew_ppm']:+.2f} ppm (from {f['slope_from']}), {f['n']} event pairs, "
                    f"spread {row['spread_us']:.0f} us; BLE-only offset differs by "
                    f"{(row['ble_offset_us'] - row['offset_us']) / 1000:+.2f} ms")
        if final and not self.events.fits and len(self.maps) > 1:
            log_msg("no event pairs yet: knock/clap near both sensors so both nodes trigger")


# ---------------------------------------------------------------------------------------------
# BLE receive (as in ble_rx.py) and offline replay
# ---------------------------------------------------------------------------------------------

class NodeStats:
    """Packet-loss counters for one ESP32. Each node has its own seq counter."""

    def __init__(self, name):
        self.name = name
        self.last_seq = None
        self.received = 0
        self.lost = 0
        self.bad = 0


class Logger:
    def __init__(self, out_path, sync: SyncEngine, quiet=False):
        self.file = open(out_path, "a", buffering=1) if out_path else None
        self.sync = sync
        self.quiet = quiet
        if not quiet:
            print(HEADER, flush=True)
        if self.file and self.file.tell() == 0:
            self.file.write(HEADER + "\n")

    def on_packet(self, stats: NodeStats, data: bytearray):
        rx_ns = now_ns()  # take the arrival time first
        fields = decode(bytes(data))
        if fields is None:
            stats.bad += 1
            if len(data) < PACKET_SIZE:
                log_msg(f"{stats.name}: short packet ({len(data)} B), the BLE MTU is too small")
            return
        seq = fields[1]
        if stats.last_seq is not None:
            gap = (seq - stats.last_seq - 1) & 0xFFFF
            if gap:
                stats.lost += gap
                log_msg(f"{stats.name}: lost {gap} packet(s) before seq {seq}")
        stats.last_seq = seq
        stats.received += 1
        line = ",".join(str(v) for v in (rx_ns, *fields))
        if not self.quiet:
            print(line, flush=True)
        if self.file:
            self.file.write(line + "\n")
        self.sync.on_packet(rx_ns, fields)


async def run_node(name, uuid, log: Logger, ble_lock: asyncio.Lock, stats: NodeStats):
    """Find one ESP32 by name, subscribe, and reconnect whenever the link drops."""
    from bleak import BleakClient, BleakScanner

    while True:
        disconnected = asyncio.Event()
        client = None

        # BlueZ handles one scan/connect at a time reliably, so the nodes take turns.
        async with ble_lock:
            log_msg(f"scanning for {name} ...")
            device = await BleakScanner.find_device_by_name(name, timeout=SCAN_TIMEOUT_S)
            if device is not None:
                client = BleakClient(device, disconnected_callback=lambda _c: disconnected.set())
                try:
                    await client.connect()
                    stats.last_seq = None  # do not count the reconnect gap as loss
                    await client.start_notify(uuid, lambda _s, d: log.on_packet(stats, d))
                    log_msg(f"connected to {name} ({device.address})")
                except Exception as e:  # BLE errors are common; retry
                    log_msg(f"{name}: BLE error: {e}")
                    try:
                        await client.disconnect()
                    except Exception:
                        pass
                    client = None

        if client is None:
            await asyncio.sleep(0.5)  # give the other node a turn at the scanner
            continue

        try:
            await disconnected.wait()
        finally:
            if client.is_connected:
                await client.disconnect()
        log_msg(f"{name} disconnected (received {stats.received}, lost {stats.lost}, "
                f"bad {stats.bad}), retrying")
        await asyncio.sleep(1.0)


async def run_live(args, sync):
    log = Logger(args.out, sync, args.quiet)
    ble_lock = asyncio.Lock()
    all_stats = [NodeStats(n) for n in args.names]
    try:
        await asyncio.gather(*(run_node(s.name, args.uuid, log, ble_lock, s) for s in all_stats))
    finally:
        for s in all_stats:
            log_msg(f"{s.name}: received {s.received}, lost {s.lost}, bad {s.bad}")
        sync.report(final=True)


def run_replay(path, sync):
    with open(path) as f:
        for line in f:
            if not line.strip() or line.startswith(("pi_rx_ns", "#")):
                continue
            v = [int(x) for x in line.strip().split(",")]
            sync.on_packet(v[0], tuple(v[1:]))
    sync.report(final=True)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--names", nargs="+", default=DEFAULT_NAMES,
                   help="BLE device names to connect to (default: %(default)s)")
    p.add_argument("--uuid", default=DATA_CHAR_UUID, help="data characteristic UUID")
    p.add_argument("--out", help="raw CSV file to append to")
    p.add_argument("--sync-out", help="sync CSV file (default: <out>.sync.csv, or stdout with --replay)")
    p.add_argument("--replay", metavar="CSV", help="run the sync on a recorded raw CSV instead of BLE")
    p.add_argument("--quiet", action="store_true", help="do not print every packet to the terminal")
    g = p.add_argument_group("sync tuning")
    g.add_argument("--ref-node", type=int, default=1, help="node whose clock is the reference (default 1 = ACU)")
    g.add_argument("--bin-s", type=float, default=5.0, help="min-delay bin length (default %(default)s s)")
    g.add_argument("--window-s", type=float, default=600.0, help="clock-map history (default %(default)s s)")
    g.add_argument("--pair-window-ms", type=float, default=30.0,
                   help="max Pi-time gap to pair two detections (default %(default)s ms)")
    g.add_argument("--refractory-ms", type=float, default=300.0,
                   help="edges closer than this to the previous edge are the same event (default %(default)s ms)")
    g.add_argument("--min-pairs-skew", type=int, default=4,
                   help="event pairs needed before drift is taken from events (default %(default)s)")
    g.add_argument("--min-span-s", type=float, default=60.0,
                   help="time span of pairs needed before drift is taken from events (default %(default)s s)")
    g.add_argument("--max-pairs", type=int, default=200, help="event pairs kept for the fit (default %(default)s)")
    g.add_argument("--report-s", type=float, default=10.0, help="summary interval (default %(default)s s)")
    args = p.parse_args()

    sync_path = args.sync_out
    if sync_path is None and args.out:
        sync_path = os.path.splitext(args.out)[0] + ".sync.csv"
    out = open(sync_path, "a", buffering=1) if sync_path else (sys.stdout if args.replay else open(os.devnull, "w"))
    sync = SyncEngine(args, out)
    try:
        if args.replay:
            run_replay(args.replay, sync)
        else:
            asyncio.run(run_live(args, sync))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
