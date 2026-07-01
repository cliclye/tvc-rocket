#!/usr/bin/env python3
"""Plot a flight log dumped from the TVC flight computer.

Usage:
  1. After a flight, connect USB, open a serial terminal at 115200,
     type `dump`, and save everything between the CSV header line and
     `EOF` to a file (e.g. flight.csv).
     Or capture directly:  python tools/plot_log.py --port /dev/cu.usbmodem* 
  2. Plot it:              python tools/plot_log.py flight.csv

Requires: pandas, matplotlib (pip install pandas matplotlib pyserial)
"""
import argparse
import sys

STATE_NAMES = {
    0: "IDLE", 1: "CAL", 2: "ARMED", 3: "BOOST", 4: "COAST",
    5: "DESCENT", 6: "LANDED", 7: "ABORT", 8: "SENSOR_FAIL",
}


def capture_from_serial(port: str, out_path: str) -> str:
    import serial  # pyserial

    with serial.Serial(port, 115200, timeout=5) as ser:
        ser.write(b"dump\n")
        lines = []
        started = False
        while True:
            line = ser.readline().decode(errors="replace").strip()
            if not line:
                break
            if line.startswith("t_ms,"):
                started = True
            if line == "EOF":
                break
            if started:
                lines.append(line)
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"Saved {len(lines) - 1} records to {out_path}")
    return out_path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", nargs="?", help="CSV file dumped from the board")
    ap.add_argument("--port", help="serial port to capture the dump from")
    args = ap.parse_args()

    path = args.csv
    if args.port:
        path = capture_from_serial(args.port, args.csv or "flight.csv")
    if not path:
        ap.error("provide a CSV file or --port")

    import matplotlib.pyplot as plt
    import pandas as pd

    df = pd.read_csv(path)
    if df.empty:
        sys.exit("Log is empty.")
    t = (df.t_ms - df.t_ms.iloc[0]) / 1000.0

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(12, 10))

    axes[0].plot(t, df.pitch_deg, label="pitch (about X)")
    axes[0].plot(t, df.yaw_deg, label="yaw (about Y)")
    axes[0].set_ylabel("tilt [deg]")
    axes[0].legend(); axes[0].grid(True)

    axes[1].plot(t, df.tvc_x_deg, label="TVC X cmd")
    axes[1].plot(t, df.tvc_y_deg, label="TVC Y cmd")
    axes[1].set_ylabel("nozzle [deg]")
    axes[1].legend(); axes[1].grid(True)

    axes[2].plot(t, df.az_g, label="axial accel (Z)")
    axes[2].plot(t, df.gx_dps / 100, label="gyro X [x100 dps]", alpha=0.6)
    axes[2].plot(t, df.gy_dps / 100, label="gyro Y [x100 dps]", alpha=0.6)
    axes[2].set_ylabel("accel [g] / rate")
    axes[2].legend(); axes[2].grid(True)

    axes[3].plot(t, df.alt_m, label="baro altitude")
    axes[3].set_ylabel("alt AGL [m]"); axes[3].set_xlabel("time [s]")
    axes[3].legend(); axes[3].grid(True)

    # Shade flight states and mark pyro events.
    changes = df.index[df.state.ne(df.state.shift())].tolist() + [len(df) - 1]
    for a, b in zip(changes[:-1], changes[1:]):
        name = STATE_NAMES.get(int(df.state.iloc[a]), "?")
        if name in ("BOOST", "DESCENT", "ABORT"):
            color = {"BOOST": "orange", "DESCENT": "lightblue",
                     "ABORT": "red"}[name]
            for ax in axes:
                ax.axvspan(t.iloc[a], t.iloc[b], alpha=0.15, color=color)
        axes[0].text(t.iloc[a], axes[0].get_ylim()[1] * 0.9, name,
                     fontsize=7, rotation=90)
    pyro = df.index[(df.flags & 1).ne((df.flags & 1).shift().fillna(0))]
    for i in pyro:
        for ax in axes:
            ax.axvline(t.iloc[i], color="red", ls="--", lw=1)

    fig.suptitle("TVC flight log")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
