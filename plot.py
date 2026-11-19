"""Plot a KY-037 CSV log produced by the logger script.

Install:  pip install pandas matplotlib
Run:      python plot_ky037.py                 (uses the newest ky037_*.csv in this folder)
          python plot_ky037.py my_data.csv     (plot a specific file)
Saves a PNG next to the CSV and also opens a window.
"""
import glob
import os
import sys

import matplotlib.pyplot as plt
import pandas as pd

if len(sys.argv) > 1:
    csv_file = sys.argv[1]
else:
    files = glob.glob("ky037_*.csv")
    if not files:
        sys.exit("No ky037_*.csv found here. Pass a filename: python plot_ky037.py my_data.csv")
    csv_file = max(files, key=os.path.getmtime)

df = pd.read_csv(csv_file, parse_dates=["timestamp"])
if df.empty:
    sys.exit(f"{csv_file} has no data rows.")

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 8), sharex=True,
                                    gridspec_kw={"height_ratios": [3, 3, 1]})

ax1.plot(df["timestamp"], df["level"], color="tab:blue", linewidth=1)
ax1.set_ylabel("Level (peak-to-peak)")
ax1.set_title(f"KY-037 sound sensor: {os.path.basename(csv_file)}")
ax1.grid(alpha=0.3)

ax2.plot(df["timestamp"], df["raw"], color="tab:orange", linewidth=0.8)
ax2.set_ylabel("Raw ADC (0-4095)")
ax2.set_ylim(0, 4095)
ax2.grid(alpha=0.3)

ax3.step(df["timestamp"], df["trigger"], where="post", color="tab:green")
ax3.set_ylabel("Trigger")
ax3.set_yticks([0, 1])
ax3.set_xlabel("Time")
ax3.grid(alpha=0.3)

fig.autofmt_xdate()
fig.tight_layout()

out_png = os.path.splitext(csv_file)[0] + ".png"
fig.savefig(out_png, dpi=150)
print(f"Saved {out_png}")
plt.show()