#!/usr/bin/env python3
"""Make a fake raw CSV (same format as ble_rx_sync.py --out) with known clock offsets and drifts,
so the sync can be tested without hardware:

    python3 simulate_nodes.py --out sim.csv
    python3 ble_rx_sync.py --replay sim.csv --sync-out sim.sync.csv

Model: each node clock = (Pi time) * (1 + drift) + offset. Packets every 50 ms of node time, delivered at
the next BLE connection event plus host latency. Knocks every 3-10 s; the ACU sees them after
--acu-latency-us, the mic after --mic-distance-m of sound travel plus its own latency.
"""

import argparse
import math
import random

WINDOW_US = 50_000


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="sim.csv")
    p.add_argument("--duration-s", type=float, default=1200)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--conn-interval-ms", type=float, default=30.0)
    p.add_argument("--acu-latency-us", type=float, default=400)
    p.add_argument("--mic-distance-m", type=float, default=0.3)
    p.add_argument("--dither-ms", type=float, default=0,
                   help="random delay before each send, as the firmware fix (0 = off)")
    args = p.parse_args()
    rnd = random.Random(args.seed)

    # node: (drift, offset_us, host latency floor us, connection phase us)
    nodes = {1: (+18e-6, 5.0e9, 1200.0, 3000.0), 2: (-12e-6, 1.2e9, 2100.0, 17000.0)}
    ci = args.conn_interval_ms * 1000
    mic_lat = args.mic_distance_m / 343.0 * 1e6 + 200.0
    lat = {1: args.acu_latency_us, 2: mic_lat}

    dur = args.duration_s * 1e6
    knocks, t = [], 2e6
    while t < dur:
        knocks.append(t)
        t += rnd.uniform(3e6, 10e6)

    rows = []
    for node, (drift, off, host, phase) in nodes.items():
        def node_t(T):
            return T * (1 + drift) + off

        def pi_t(tn):
            return (tn - off) / (1 + drift)

        edges = []
        for k in knocks:
            if rnd.random() < 0.05:
                continue  # missed detection
            e = node_t(k + lat[node] + rnd.gauss(0, 60))
            edges.append(e)
            if node == 2 and rnd.random() < 0.5:
                edges.append(e + 60_000)  # echo in the next window
        n_false = int(args.duration_s / 30)
        edges += [node_t(rnd.uniform(0, dur)) for _ in range(n_false)]  # false triggers
        edges.sort()

        tn = math.ceil(node_t(0) / WINDOW_US) * WINDOW_US
        end = node_t(dur)
        seq, ei, last_ev = 0, 0, 0.0
        while tn < end:
            w_end = tn + WINDOW_US
            while ei < len(edges) and edges[ei] < tn:
                ei += 1
            edge = int(edges[ei]) if ei < len(edges) and edges[ei] < w_end else 0
            tx = int(w_end + rnd.uniform(0, args.dither_ms * 1000 if args.dither_ms else 300))
            T_tx = pi_t(tx)
            T_ev = phase + math.ceil((T_tx + 300 - phase) / ci) * ci  # next connection event
            if rnd.random() < 0.05:
                T_ev += ci  # retransmission
            T_ev = max(T_ev, last_ev)  # BLE delivers in order: a retransmission delays later packets
            last_ev = T_ev
            rx = T_ev + host + rnd.expovariate(1 / 400)
            if rnd.random() > 0.01:  # 1 % lost
                rows.append((int(rx * 1000), node, seq & 0xFFFF, edge, edge, tx, 0, 0, int(edge > 0)))
            seq += 1
            tn = w_end
    rows.sort()
    with open(args.out, "w") as f:
        f.write("pi_rx_ns,node,seq,t_edge_us,t_peak_us,tx_us,raw,level,trigger\n")
        for r in rows:
            f.write(",".join(map(str, r)) + "\n")

    d1, o1 = nodes[1][:2]
    d2, o2 = nodes[2][:2]
    rel = (1 + d2) / (1 + d1) - 1
    t_ref = (dur * (1 + d1) + o1)
    T = (t_ref - o1) / (1 + d1)
    true_off = T * (1 + d2) + o2 - t_ref
    bias = lat[2] * (1 + d2) - lat[1] * (1 + d1)
    print(f"wrote {len(rows)} packets, {len(knocks)} knocks")
    print(f"truth: Sound-ACU drift {rel * 1e6:+.3f} ppm; offset at end {true_off / 1000:+.3f} ms "
          f"(event method should read {(true_off + bias) / 1000:+.3f} ms: +{bias:.0f} us sensor-latency difference)")
    print(f"truth: ACU vs Pi drift {(1 / (1 + d1) - 1) * 1e6:+.3f} ppm, Sound vs Pi {(1 / (1 + d2) - 1) * 1e6:+.3f} ppm")


if __name__ == "__main__":
    main()
