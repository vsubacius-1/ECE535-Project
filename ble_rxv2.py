#!/usr/bin/env python3
"""Receive sensor packets from both ESP32 nodes over BLE and log them as one CSV.

Nodes (both use the same service/characteristic UUIDs and the same 33-byte packet):
  TSN-Sound  node 2  KY-037 microphone
  TSN-ACU    node 1  ADXL345 accelerometer

Install:  pip install bleak            (setup_ble.sh does this in ~/tsn-venv)
Run:      python3 ble_rx.py                          # both nodes, print to terminal
          python3 ble_rx.py --out run.csv            # also save to a file
          python3 ble_rx.py --names TSN-ACU          # only one node

Columns:
  pi_rx_ns   Pi monotonic clock when the packet arrived (ns)
  node       1 = ACU (ADXL345), 2 = sound (KY-037)
  seq        packet counter from that ESP32 (wraps at 65535)
  t_edge_us  ESP32 clock at the first event edge in the 50 ms window, 0 = none
             (sound: KY-037 DO edge; ACU: ADXL345 activity interrupt on INT2)
  t_peak_us  ESP32 clock at the largest sample in the window
  tx_us      ESP32 clock just before the packet was sent
  raw        sound: last AO reading (0-4095);  ACU: |a| in mg
  level      sound: noise-subtracted level;    ACU: peak-to-peak |a| in mg
  trigger    1 if t_edge_us is set

Each ESP32 has its own clock, so compare t_* values only within one node,
or after the sync algorithm has estimated the offset between the nodes.
"""

import argparse
import asyncio
import struct
import sys
import time

from bleak import BleakClient, BleakScanner

DEFAULT_NAMES = ["TSN-Sound", "TSN-ACU"]

DATA_CHAR_UUID = "0E76AAE8-1EE4-456A-9708-DE09E78F4BCA"

PACKET_FMT = "<BBHQQQHHB"  # little-endian, no padding
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # 33
PACKET_VERSION = 1
HEADER = "pi_rx_ns,node,seq,t_edge_us,t_peak_us,tx_us,raw,level,trigger"
SCAN_TIMEOUT_S = 5.0


def log_msg(msg):
    print(f"# {msg}", file=sys.stderr, flush=True)


def decode(data: bytes):
    """Return the packet fields (without version) as a tuple, or None if not valid."""
    if len(data) != PACKET_SIZE:
        return None
    fields = struct.unpack(PACKET_FMT, data)
    if fields[0] != PACKET_VERSION:
        return None
    return fields[1:]


class NodeStats:
    """Packet-loss counters for one ESP32. Each node has its own seq counter."""

    def __init__(self, name):
        self.name = name
        self.last_seq = None
        self.received = 0
        self.lost = 0
        self.bad = 0


class Logger:
    def __init__(self, out_path):
        self.file = open(out_path, "a", buffering=1) if out_path else None
        print(HEADER, flush=True)
        if self.file and self.file.tell() == 0:
            self.file.write(HEADER + "\n")

    def on_packet(self, stats: NodeStats, data: bytearray):
        rx_ns = time.monotonic_ns()  # take the arrival time first
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
        print(line, flush=True)
        if self.file:
            self.file.write(line + "\n")


async def run_node(name, uuid, log: Logger, ble_lock: asyncio.Lock, stats: NodeStats):
    """Find one ESP32 by name, subscribe, and reconnect whenever the link drops."""
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


async def run(args):
    log = Logger(args.out)
    ble_lock = asyncio.Lock()
    all_stats = [NodeStats(n) for n in args.names]
    try:
        await asyncio.gather(*(run_node(s.name, args.uuid, log, ble_lock, s) for s in all_stats))
    finally:
        for s in all_stats:
            log_msg(f"{s.name}: received {s.received}, lost {s.lost}, bad {s.bad}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--names", nargs="+", default=DEFAULT_NAMES,
                   help="BLE device names to connect to (default: %(default)s)")
    p.add_argument("--uuid", default=DATA_CHAR_UUID, help="data characteristic UUID")
    p.add_argument("--out", help="CSV file to append to")
    try:
        asyncio.run(run(p.parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
