#!/usr/bin/env python3
"""Receive KY-037 sound packets from the ESP32 over BLE and log them as CSV.

Install:  pip install bleak
Run:      python3 ble_rx.py                 # print to the terminal
          python3 ble_rx.py --out sound.csv # also save to a file

Columns:
  pi_rx_ns   Pi monotonic clock when the packet arrived (ns)
  seq        packet counter from the ESP32 (wraps at 65535)
  t_edge_us  ESP32 clock at the first DO edge in the window, 0 = none
  t_peak_us  ESP32 clock at the loudest AO sample in the window
  tx_us      ESP32 clock just before the packet was sent
  raw, level, trigger   same values as the USB serial output

The name, UUIDs and packet layout must match ky037_esp32_ble.ino.
"""

import argparse
import asyncio
import struct
import sys
import time

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "TSN-Sound"
DATA_CHAR_UUID = "8ef8eb63-4406-4d27-b44e-b41ce16723f2"

PACKET_FMT = "<BBHQQQHHB"  # little-endian, no padding
PACKET_SIZE = struct.calcsize(PACKET_FMT)  # 33
PACKET_VERSION = 1
HEADER = "pi_rx_ns,node,seq,t_edge_us,t_peak_us,tx_us,raw,level,trigger"


def decode(data: bytes):
    """Return the packet fields as a tuple, or None if the packet is not valid."""
    if len(data) != PACKET_SIZE:
        return None
    fields = struct.unpack(PACKET_FMT, data)
    if fields[0] != PACKET_VERSION:
        return None
    return fields[1:]  # drop version


class Logger:
    def __init__(self, out_path):
        self.file = open(out_path, "a", buffering=1) if out_path else None
        self.last_seq = None
        self.lost = 0
        self.bad = 0
        print(HEADER)
        if self.file and self.file.tell() == 0:
            self.file.write(HEADER + "\n")

    def on_packet(self, _sender, data: bytearray):
        rx_ns = time.monotonic_ns()  # take the arrival time first
        fields = decode(bytes(data))
        if fields is None:
            self.bad += 1
            if len(data) < PACKET_SIZE:
                print(f"# short packet ({len(data)} B): the BLE MTU is too small", file=sys.stderr)
            return
        seq = fields[1]
        if self.last_seq is not None:
            gap = (seq - self.last_seq - 1) & 0xFFFF
            if gap:
                self.lost += gap
                print(f"# lost {gap} packet(s) before seq {seq}", file=sys.stderr)
        self.last_seq = seq
        line = ",".join(str(v) for v in (rx_ns, *fields))
        print(line)
        if self.file:
            self.file.write(line + "\n")


async def run(args):
    log = Logger(args.out)
    while True:
        print(f"# scanning for {args.name} ...", file=sys.stderr)
        device = await BleakScanner.find_device_by_name(args.name, timeout=10.0)
        if device is None:
            continue

        disconnected = asyncio.Event()
        try:
            async with BleakClient(device, disconnected_callback=lambda _c: disconnected.set()) as client:
                print(f"# connected to {device.address}", file=sys.stderr)
                log.last_seq = None  # do not count the reconnect gap as loss
                await client.start_notify(args.uuid, log.on_packet)
                await disconnected.wait()
        except Exception as e:  # BLE errors are common; retry
            print(f"# BLE error: {e}", file=sys.stderr)
        print(f"# disconnected (lost so far: {log.lost}, bad: {log.bad}), retrying", file=sys.stderr)
        await asyncio.sleep(1.0)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--name", default=DEVICE_NAME, help="BLE device name")
    p.add_argument("--uuid", default=DATA_CHAR_UUID, help="data characteristic UUID")
    p.add_argument("--out", help="CSV file to append to")
    try:
        asyncio.run(run(p.parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
