#!/usr/bin/env python3
"""Send a truth pulse to both ESP32 nodes at a fixed interval (evaluation only).

Both nodes see the same rising edge at the same instant. Each one timestamps it
and sends it over BLE in a packet with trigger = 2. sync.py pairs the two
timestamps: their difference is the true clock offset.

Wiring:
  Pi GPIO17 (pin 11) -> GPIO33 on the ACU ESP32  AND  GPIO33 on the sound ESP32
  Pi GND    (pin 9)  -> GND on both ESP32s       (required: common ground)

Run it in its own terminal, with the system Python (gpiozero is not in the venv):
  /usr/bin/python3 truth_pulse.py                 # one pulse every 10 s until Ctrl-C
  /usr/bin/python3 truth_pulse.py --every 5 --pin 17

Needs gpiozero. Raspberry Pi OS has it. On Ubuntu:
  sudo apt install python3-gpiozero python3-lgpio
If gpiozero is missing, the script uses the `pinctrl` command (Raspberry Pi OS Bookworm).
"""

import argparse
import shutil
import subprocess
import sys
import time


class GpiozeroPin:
    def __init__(self, pin):
        from gpiozero import DigitalOutputDevice
        self.dev = DigitalOutputDevice(pin, initial_value=False)

    def set(self, high):
        self.dev.value = 1 if high else 0

    def close(self):
        self.dev.off()
        self.dev.close()


class PinctrlPin:
    def __init__(self, pin):
        self.pin = str(pin)
        self.set(False)

    def set(self, high):
        subprocess.run(["pinctrl", "set", self.pin, "op", "dh" if high else "dl"], check=True)

    def close(self):
        self.set(False)


def open_pin(pin):
    try:
        return GpiozeroPin(pin), "gpiozero"
    except Exception as e:  # not installed, or no GPIO backend
        if shutil.which("pinctrl"):
            return PinctrlPin(pin), "pinctrl"
        sys.exit(f"Cannot drive GPIO{pin}: gpiozero failed ({e}) and `pinctrl` is not installed.\n"
                 "Install: sudo apt install python3-gpiozero python3-lgpio")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pin", type=int, default=17, help="BCM GPIO number (default 17 = pin 11)")
    p.add_argument("--every", type=float, default=10.0, help="seconds between pulses (default 10)")
    p.add_argument("--width-ms", type=float, default=10.0, help="pulse width in ms (default 10)")
    p.add_argument("--count", type=int, default=0, help="stop after this many pulses (0 = run until Ctrl-C)")
    args = p.parse_args()
    if args.every < 1.0:
        sys.exit("--every must be at least 1 s, so sync.py can tell the pulses apart")

    pin, how = open_pin(args.pin)
    print(f"# truth pulse on GPIO{args.pin} via {how}: {args.width_ms:.0f} ms every {args.every:g} s. "
          "Ctrl-C to stop.", file=sys.stderr, flush=True)
    n = 0
    next_t = time.monotonic() + 1.0
    try:
        while args.count == 0 or n < args.count:
            time.sleep(max(0.0, next_t - time.monotonic()))
            pin.set(True)
            time.sleep(args.width_ms / 1000.0)
            pin.set(False)
            n += 1
            print(f"# pulse {n}", file=sys.stderr, flush=True)
            next_t += args.every
    except KeyboardInterrupt:
        pass
    finally:
        pin.close()
        print(f"# sent {n} pulse(s)", file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
