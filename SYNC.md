# Sync algorithm: hybrid Kalman filter

`sync.py` calculates the clock offset and drift between the two ESP32 nodes. It reads the
CSV that `ble_rx.py` writes. It runs four methods on the same data, so you can compare them:

| Method | What it uses | Purpose |
|---|---|---|
| `haest` | Event pairs. HAEST Section V-A: k-means (k = 20), mean of cluster slopes, smallest cluster | Baseline from the paper |
| `packets` | Packet timing only (`tx_us` and Pi arrival time) | Baseline without events |
| `ours_events` | Our Kalman filter, event pairs only | Ablation |
| **`ours_hybrid`** | **Our Kalman filter, packets and event pairs** | **Our method** |

Offset means `t_sound − t_acu` for one physical instant, in µs. Drift is in ppm (µs per s).

## Files

| File | Purpose |
|---|---|
| `sync.py` | The algorithm, offline replay, live mode, plots |
| `sync_sim.py` | Simulator. It writes a CSV in the `ble_rx.py` format, plus the true offsets |
| `sync_eval.py` | Runs all methods on 7 scenarios and prints a Markdown table for the report |
| `test_sync.py` | 20 tests. Run `python3 -m unittest test_sync -v` |
| `truth_pulse.py` | Sends the truth pulse to both ESP32s (evaluation only, see "Truth") |
| `sync_example.png` | Example plot from a simulated run |

Only `sync.py` is needed on the Pi for a run, plus `truth_pulse.py` if you use the truth pulse.
The sketches for the truth pulse are `ky037_esp32_blev3.ino` (sound) and
`acu_esp32_blev2.ino` (ACU).

You need `numpy`. For `--plot`, you also need `matplotlib`. Install them in the same venv as
`bleak`: `~/tsn-venv/bin/pip install numpy matplotlib`.

## Quick start

Try it on simulated data first:

```bash
python3 sync_sim.py --out sim_run.csv --truth sim_truth.csv
# it prints a line like: use: --bias-us -984
python3 sync.py sim_run.csv --truth sim_truth.csv --bias-us -984 --plot sync.png --out est.csv
python3 sync_eval.py            # comparison table, about 30 s
```

On real data:

```bash
python3 ble_rx.py --out run.csv                     # record (Ctrl-C to stop)
python3 sync.py run.csv --bias-us <your value> --plot sync.png --out est.csv
python3 sync.py run.csv --follow --bias-us <value>  # live, in a 2nd terminal while ble_rx.py runs
```

The live mode prints the offset, the drift, the 1-sigma error and the CPU use every 5 s.

## How the hybrid method works

The method has four stages. All stages run on the Pi. The ESP32s send only their normal packets.

**1. Coarse alignment (packets).** Each packet has `tx_us` (node clock) and `pi_rx_ns` (Pi clock).
`tx − pi_rx` is the node offset minus the BLE delay. The delay is always positive. Thus the
maximum of `tx − pi_rx` in a 2 s bin is the node offset minus the smallest delay. The
pipeline uses this value to put each event on the Pi clock. It is accurate to a few ms only.

**2. Event pairing.** An event is the first edge after 250 ms of quiet on one node. The
pipeline pairs an ACU event with a sound event if their Pi times are within ±50 ms. After
5 pairs, the window becomes ±10 ms, centred on the median of the recent pairs. If a window
has two candidates, the pipeline discards the event.

Option: `--pairing pattern` pairs the events by the gaps between them, like a fingerprint.
It does not use packet timing at all. In each 2-minute window, it finds the offset that lines
up the most events. In the simulator, it finds the same pairs as stage 2. Taps must not be
perfectly periodic.

**3. Kalman filter.** The state is:

| State | Meaning |
|---|---|
| θ | Offset of the reference event group, µs |
| α | Drift, ppm |
| b_pkt | Constant bias of the packet measurement. This is the difference between the minimum BLE delays of the two links. |
| b_g | Bias of event group g, relative to the reference group. The reference group has b = 0. |

The filter gets two types of measurement:

- Every 10 s, one **packet measurement**: `z = θ + b_pkt`. Its noise is large, but it never
  stops. Over minutes, it gives the drift very accurately. It cannot give the offset,
  because b_pkt is not known.
- For each pair, one **event measurement**: `z = θ + b_g`. It is accurate, but it does not
  come at a fixed rate. It gives the offset.

**4. Event groups.** Taps at different positions have different sound delays (2.9 ms per
metre). Each position becomes one group with its own bias. A new event joins the group that
predicts it best, if the error is less than 3 sigma. Other events wait. When 3 waiting
events lie on one line (±500 µs) with a drift that the packets allow (α ± 3σ), they become a
new group.

The group with the smallest bias is the reference. It has the shortest sound path. If a new
group is lower than the reference, it becomes the new reference. The filter changes the
state to match, and no information is lost. Then the output is `θ − bias_us`.

Each event pair has a weight from its amplitude. A weak tap has more timing jitter, so the
filter trusts it less.

### What is new compared to HAEST

| HAEST (Section V-A) | Our method | Why |
|---|---|---|
| k-means on raw offsets | Groups are lines in (time, offset): drift is removed first | Drift moves the offset, for example 12 ms in 10 min at 20 ppm. Then k-means splits one position into many clusters. |
| Fixed k = 20, drop clusters < 3 | Groups form when 3 events agree. No k. | No guessed parameter. |
| Drift = mean of the cluster slopes | One drift for all groups, in one filter, plus the packets | A small cluster gives a noisy slope, but it counts as much as a large one. |
| Batch, 10-minute window | Recursive. Each update is a few small matrix operations | It follows drift changes, and it gives an error bound at all times. |
| Events only | Packets give the drift, events give the offset | The method keeps working with few events. |
| Offset = cluster with the smallest mean | Calibrated reference group, automatic re-reference | The propagation delay is removed, not only made smaller. |
| Window filter drops crowded windows | Same, plus a narrow adaptive window and amplitude weights | Fewer false pairs. |

## Results on the simulator

Mean of 5 seeds. All errors are in µs. The goal is a mean absolute error (MAE) below 1000 µs.
Lock is the first event-based estimate (the `packets` method never locks).

| scenario | method | coverage | MAE whole run | lock (s) | MAE after lock | worst p95 after lock |
|---|---|---|---|---|---|---|
| baseline | haest | 89% | 411 | 132 | 411 | 1154 |
|  | packets | 99% | 2331 | - | - | - |
|  | ours_events | 98% | 54 | 32 | 54 | 151 |
|  | **ours_hybrid** | 99% | 57 | 26 | 48 | 142 |
| low_rate | haest | 0% | - | - | - | - |
|  | packets | 100% | 2349 | - | - | - |
|  | ours_events | 64% | 565 | 862 | 565 | 5522 |
|  | **ours_hybrid** | 100% | 650 | 612 | 94 | 308 |
| drift_step | haest | 89% | 350 | 132 | 350 | 1154 |
|  | packets | 99% | 2486 | - | - | - |
|  | ours_events | 98% | 114 | 32 | 114 | 403 |
|  | **ours_hybrid** | 99% | 113 | 26 | 104 | 374 |
| late_nearest | haest | 90% | 991 | 128 | 991 | 2878 |
|  | packets | 99% | 2331 | - | - | - |
|  | ours_events | 99% | 784 | 26 | 784 | 2289 |
|  | **ours_hybrid** | 99% | 787 | 24 | 783 | 2280 |
| busy_room | haest | 86% | 445 | 172 | 445 | 1106 |
|  | packets | 99% | 2338 | - | - | - |
|  | ours_events | 98% | 61 | 34 | 61 | 172 |
|  | **ours_hybrid** | 99% | 71 | 30 | 55 | 156 |
| pattern_pairing | haest | 89% | 412 | 132 | 412 | 1154 |
|  | packets | 99% | 2331 | - | - | - |
|  | ours_events | 98% | 54 | 32 | 54 | 151 |
|  | **ours_hybrid** | 99% | 57 | 26 | 48 | 142 |
| no_weights | haest | 89% | 411 | 132 | 411 | 1154 |
|  | packets | 99% | 2331 | - | - | - |
|  | ours_events | 98% | 63 | 32 | 63 | 242 |
|  | **ours_hybrid** | 99% | 68 | 26 | 58 | 211 |

Scenarios: **baseline** 1 tap per ~4 s from 3 positions (0.15, 0.8, 1.6 m from the mic), 20 min.
**low_rate** 1 tap per ~2 min, 40 min. **drift_step** +5 ppm at 10 min. **late_nearest** the
nearest position is used only after 7 min. **busy_room** 5× more one-sensor ambient events
and 5% double taps. **pattern_pairing** stage 2 replaced by `--pairing pattern`.
**no_weights** amplitude weights off.

What the table shows:

- **Accuracy.** After lock, our filter is about 8× more accurate than the HAEST baseline:
  about 50 µs against about 400 µs. It also locks in about 30 s, against about 130 s.
- **HAEST drift.** In the baseline run, HAEST estimates −2 to −5 ppm. The true drift is
  −20 ppm (see `sync_example.png`). This is the k-means problem in the table above. HAEST
  still gets a fair offset, because it extrapolates from its newest cluster.
- **Low event rate.** HAEST gives no estimate, and events-only covers 64% of the run with a
  bad p95. The hybrid always gives an estimate: packet sync (about 2.3 ms) until 3 taps agree,
  then about 94 µs. This is the main gain from the packets.
- **Packets alone.** The error is 2.3 ms, which is the difference of the two BLE stack delays.
  The packets give the drift, not the offset. This is why the method needs events.
- **late_nearest.** Before the nearest position is used, all methods have about 2 ms error.
  That is the bias difference between positions. After that, the hybrid re-references and
  falls to about 50 µs (HAEST: about 400 µs). Thus, calibrate at the tap position closest to the mic.
- **Weights** give a small gain: 58 µs against 48 µs.
- **CPU.** All four methods together use about 0.03% of one core of this PC, in real time.
  A Pi 4 is about 5–10× slower, which gives less than 0.5%. Measure it on the Pi with `--follow`.

**Limits of the simulator.** The hardware has not run this code yet. The BLE delay model,
jitter values and stack delays are estimates. The simulator's ESP32 has no interrupt
latency. Do the hardware runs before you put these numbers in the report.

## Calibration: `--bias-us`

The event offset is `t_sound − t_acu = clock offset + (sound delay − ACU delay)`. The bias is
the second part. It includes the sound travel time, the ADXL345 filter delay and the KY-037
comparator delay. Drift is not affected by it. The offset is.

Do the calibration once with the logic analyzer:

1. Connect the logic analyzer to ADXL345 INT2 and KY-037 DO.
2. Tap 20 times at the reference position. Use the position closest to the microphone.
3. For each tap, measure `DO edge − INT edge` on the logic analyzer.
4. Use the mean as `--bias-us`. A negative value is correct if the ACU edge comes later.

If you do not calibrate (`--bias-us 0`), the drift is still correct. The offset is then
wrong by the bias, about 1 ms in the simulator.

## Truth for the evaluation

Truth is the real offset `t_sound − t_acu` at one physical instant. `sync.py` compares its
estimate to it.

### Method A: truth pulse (recommended)

The Pi sends one pulse to both ESP32s at the same instant. Each ESP32 timestamps the rising
edge in an interrupt and sends the time over BLE in one extra packet with `trigger = 2`.
The difference of the two timestamps is the true offset, to within a few µs.

```
   Pi GPIO17 (pin 11) ──────►  GPIO33 on the ACU ESP32
                       └────►  GPIO33 on the sound ESP32
   Pi GND (pin 9) ──────────►  GND on both ESP32s   (required)

   ESP32s ── BLE (normal packets + one trigger = 2 packet per pulse) ──► Pi
```

The pulse only gives a timing edge. All data goes over BLE, as before. BLE delay does not
matter, because it changes when a timestamp arrives, not its value.

Order for one run (three terminals on the Pi):

```bash
python3 ble_rxv2.py --out run1.csv > /dev/null       # 1. record; wait for both "connected"
/usr/bin/python3 truth_pulse.py                       # 2. pulse every 10 s (system Python)
python3 sync.py run1.csv --follow --bias-us <value>   # 3. optional: live error vs truth
# ... do the experiment, then Ctrl-C in all terminals ...
python3 sync.py run1.csv --bias-us <value> --plot run1.png --out run1_est.csv
```

`sync.py` finds the pulse packets in the CSV and uses them as truth automatically. It prints
`Truth: N truth-pulse pairs from the CSV`. Pulse packets are never used as events.

Notes:
- GPIO33 has a pull-down in the sketches. If the wire is not connected, nothing happens.
- `truth_pulse.py` needs gpiozero (Raspberry Pi OS has it; on Ubuntu:
  `sudo apt install python3-gpiozero python3-lgpio`). Without it, the script uses `pinctrl`.
- The pulse is for evaluation only. It is not part of the sync method, so the
  "no extra sync traffic" goal still holds.
- Do not use pulse data from the same run to calibrate `--bias-us`. Calibrate in a
  separate run, or with the logic analyzer.

### Method B: logic analyzer (no firmware change)

`--truth` takes a CSV with one row per instant: `t_acu_us,t_sound_us`. These are both node
clocks at the same physical instant. To make it from the logic analyzer recording:

1. Record INT2 and DO on the logic analyzer for the whole run.
2. Match each logic-analyzer edge pair to its event pair in the CSV, by order.
3. Write one row: `t_acu_us = t_edge(ACU)`, `t_sound_us = t_edge(sound) − (DO edge − INT2 edge)`.

This removes the sensor delays. What is left is the clock offset, plus a few µs of interrupt
latency. `--truth FILE` overrides the pulse truth.

## Parameters

All parameters are in `Config` at the top of `sync.py`. Each one is also a command-line flag,
for example `--sigma-ev-us 400`.

| Parameter | Default | Change it if |
|---|---|---|
| `bias_us` | 0 | Always. Set the calibrated value. |
| `holdoff_ms` | 250 | One tap gives two events: increase it. |
| `pair_window_ms` | 50 | Pairs are missing at the start: increase it. |
| `sigma_ev_us` | 300 | Hardware event jitter is larger: increase it. |
| `sigma_pkt_us` | 500 | Packet bins are noisier on the Pi: increase it. |
| `q_alpha` | 0.002 | Drift changes faster (temperature): increase it. |
| `group_eps_us` | 500 | Tap positions are closer than 1 ms: decrease it. |
| `kf_bin_s` | 10 | Packets are less frequent than 1 per 50 ms: increase it. |

## Known limits

1. **Reboot.** If a node reboots, `load_csv` keeps only the data after the last reboot. The
   live mode does not handle reboots. Restart it.
2. **The reference group must be the calibrated position.** The filter uses the lowest
   group. If you calibrate a different position, the offset is wrong by the difference.
3. **3 taps to lock.** The hybrid needs 3 taps from one position before it leaves packet mode.
4. **Packet bins expect about 1 packet per 50 ms per node.** If one node sends less often,
   increase `kf_bin_s` or decrease `kf_bin_min_frac`.
5. **Regular taps** can confuse `--pairing pattern`. Tap at irregular intervals.
