"""Read KY-037 data from an ESP32 over serial and print it with a live level bar.
 
Install:  pip install pyserial
Run:      python read_ky037.py COM8            (Windows)
          python read_ky037.py /dev/ttyUSB0    (Linux)
          python read_ky037.py /dev/cu.usbserial-0001   (macOS)
"""
import csv
import sys
from datetime import datetime
 
import serial
 
PORT = sys.argv[1] if len(sys.argv) > 1 else "COM8"
CSV_FILE = sys.argv[2] if len(sys.argv) > 2 else f"ky037_{datetime.now():%Y%m%d_%H%M%S}.csv"
BAUD = 115200
MAX_LEVEL = 2000  # peak-to-peak value that fills the bar; adjust to taste
 
# Configure DTR/RTS BEFORE opening so the ESP32 isn't reset when the port opens.
ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.timeout = 1
ser.dtr = False
ser.rts = False
ser.open()
 
with ser, open(CSV_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "raw", "level", "trigger"])
    print(f"Listening on {PORT}, logging to {CSV_FILE} (Ctrl+C to stop)")
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line or line.startswith("raw"):
                continue
            try:
                raw, level, trigger = (int(x) for x in line.split(","))
            except ValueError:
                continue
 
            writer.writerow([datetime.now().isoformat(timespec="milliseconds"), raw, level, trigger])
            f.flush()  # keep the file up to date even if the script is killed
 
            bar = "#" * min(40, int(level / MAX_LEVEL * 40))
            print(f"raw={raw:4d}  level={level:4d}  trigger={trigger}  |{bar:<40}|")
    except KeyboardInterrupt:
        print(f"\nStopped. Data saved to {CSV_FILE}")