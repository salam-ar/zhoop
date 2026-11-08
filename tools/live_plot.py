#!/usr/bin/env python3
"""Live-plot the ESP32 LSM6 IMU CSV stream.

Usage:
    pip install pyserial matplotlib
    python tools/live_plot.py                 # auto-detect port
    python tools/live_plot.py /dev/cu.usbserial-0001 --window 10

Lines starting with '#' (startup info) are echoed to the terminal; plotting
starts once the CSV header 't_ms,ax,ay,az,gx,gy,gz,temp_c' arrives.
Close the window or press Ctrl+C to quit.
"""
import argparse
import collections
import sys
import threading
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial
from serial.tools import list_ports

HEADER = "t_ms,ax,ay,az,gx,gy,gz,temp_c"
RATE_HZ = 104

# Validated categorical palette (blue, orange, aqua). Same color = same axis in
# both panels, so X/Y/Z read consistently between accel and gyro.
AXIS_COLORS = ["#2a78d6", "#eb6834", "#1baf7a"]
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e6e5e0"


def guess_port():
    hints = ("usbserial", "SLAB", "wchusb", "CP210", "CH340", "ttyUSB", "ttyACM")
    for p in list_ports.comports():
        if any(h.lower() in (p.device + " " + (p.description or "")).lower() for h in hints):
            return p.device
    return None


class Reader(threading.Thread):
    def __init__(self, ser, maxlen):
        super().__init__(daemon=True)
        self.ser = ser
        self.lock = threading.Lock()
        self.t = collections.deque(maxlen=maxlen)
        self.cols = [collections.deque(maxlen=maxlen) for _ in range(7)]
        self.streaming = False
        self.stop = False

    def run(self):
        while not self.stop:
            try:
                line = self.ser.readline().decode("ascii", "replace").strip()
            except serial.SerialException as e:
                print(f"serial error: {e}", file=sys.stderr)
                return
            if not line:
                continue
            if line.startswith("#"):
                print(line)
                continue
            if line == HEADER:
                self.streaming = True
                print(line)
                continue
            if not self.streaming:
                continue
            parts = line.split(",")
            if len(parts) != 8:
                continue
            try:
                vals = [float(x) for x in parts]
            except ValueError:
                continue
            with self.lock:
                self.t.append(vals[0] / 1000.0)
                for i in range(7):
                    self.cols[i].append(vals[i + 1])

    def snapshot(self):
        with self.lock:
            return list(self.t), [list(c) for c in self.cols]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=10.0, help="seconds of history to show")
    args = ap.parse_args()

    port = args.port or guess_port()
    if not port:
        ports = ", ".join(p.device for p in list_ports.comports()) or "none"
        sys.exit(f"No ESP32 serial port found (available: {ports}). Pass the port explicitly.")
    print(f"Opening {port} @ {args.baud}")
    # ESP32 DevKits wire RTS -> EN (reset) and DTR -> GPIO0 (boot mode).
    # pyserial asserts both on open, which can hold the chip in reset so
    # nothing is ever received. Deassert them before opening, then pulse RTS
    # for a clean reset so the startup banner and calibration are seen.
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, args.baud, 1
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.rts = True
    time.sleep(0.1)
    ser.rts = False
    print("Board reset - keep it still for ~3 s while the gyro calibrates.")

    reader = Reader(ser, maxlen=int(args.window * RATE_HZ * 1.2))
    reader.start()

    plt.rcParams.update({"axes.edgecolor": GRID, "axes.labelcolor": MUTED,
                         "xtick.color": MUTED, "ytick.color": MUTED, "text.color": INK})
    fig, (ax_a, ax_g) = plt.subplots(2, 1, sharex=True, figsize=(10, 6.5))
    fig.canvas.manager.set_window_title(f"LSM6 IMU - {port}")
    panels = [
        (ax_a, "Acceleration", "g", ["ax", "ay", "az"], (-1.5, 1.5)),
        (ax_g, "Angular rate", "dps", ["gx", "gy", "gz"], (-50, 50)),
    ]
    lines = []
    for ax, title, unit, names, ylim in panels:
        ax.set_title(f"{title} ({unit})", loc="left", fontsize=11, color=INK)
        ax.set_ylim(*ylim)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.axhline(0, color=MUTED, linewidth=0.8)
        for spine in ("top", "right"):
            ax.spines[spine].set_visible(False)
        for name, color in zip(names, AXIS_COLORS):
            (ln,) = ax.plot([], [], color=color, linewidth=1.6, label=name)
            lines.append(ln)
    ax_g.set_xlabel("time (s)")
    ax_g.set_xlim(0, args.window)
    legends = [ax.legend(loc="upper left", ncol=3, frameon=False, fontsize=9) for ax in (ax_a, ax_g)]
    status = fig.text(0.99, 0.01, "waiting for CSV header...", ha="right", va="bottom",
                      fontsize=9, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 1))

    def autoscale(ax, series, floor):
        vals = [v for s in series for v in s]
        if not vals:
            return
        lo, hi = min(vals), max(vals)
        span = max(hi - lo, floor)
        mid = (hi + lo) / 2
        cur_lo, cur_hi = ax.get_ylim()
        want_lo, want_hi = mid - span * 0.6, mid + span * 0.6
        # Only expand, or shrink when the data uses < 40% of the view, to avoid jitter.
        if lo < cur_lo or hi > cur_hi or (hi - lo) < 0.4 * (cur_hi - cur_lo):
            ax.set_ylim(want_lo, want_hi)

    def update(_):
        t, cols = reader.snapshot()
        if not t:
            return lines
        for ln, data in zip(lines, cols[:6]):
            ln.set_data(t, data)
        t_end = t[-1]
        ax_a.set_xlim(max(0.0, t_end - args.window), max(t_end, args.window))
        autoscale(ax_a, cols[0:3], 0.5)
        autoscale(ax_g, cols[3:6], 10.0)
        # Live values in the legend labels: identity never relies on color alone.
        names = ["ax", "ay", "az", "gx", "gy", "gz"]
        for i, txt in enumerate(legends[0].get_texts() + legends[1].get_texts()):
            fmt = "{:s} {:+.3f}" if i < 3 else "{:s} {:+.1f}"
            txt.set_text(fmt.format(names[i], cols[i][-1]))
        ax_, ay_, az_ = cols[0][-1], cols[1][-1], cols[2][-1]
        amag = (ax_ * ax_ + ay_ * ay_ + az_ * az_) ** 0.5
        status.set_text(f"|a| = {amag:.3f} g    T = {cols[6][-1]:.1f} °C    t = {t_end:.1f} s")
        return lines

    anim = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop = True
        ser.close()


if __name__ == "__main__":
    main()
