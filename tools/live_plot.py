#!/usr/bin/env python3
"""Live-plot the ESP32 LSM6 IMU CSV stream.

Usage:
    pip install pyserial matplotlib
    python tools/live_plot.py                 # auto-detect port
    python tools/live_plot.py /dev/cu.usbserial-0001 --window 10

Lines starting with '#' (startup info) are echoed to the terminal; plotting
starts once the CSV header 't_ms,ax,ay,az,gx,gy,gz,temp_c' arrives.
Close the window or press Ctrl+C to quit.

Recording a test: type a name (and optionally a duration in seconds) in the
bar under the plot, then press "Start test". Press "Stop & save" (or let the
duration run out). Every sample is written to recordings/<date>_<time>_<name>.csv
along with a .png overview of the whole test. Load the CSV with
pandas.read_csv(path, comment="#").
"""
import argparse
import collections
import datetime as dt
import re
import sys
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.widgets import Button, TextBox
import serial
from serial.tools import list_ports

HEADER = "t_ms,ax,ay,az,gx,gy,gz,temp_c"
RATE_HZ = 104

# Validated categorical palette (blue, orange, aqua). Same color = same axis in
# both panels, so X/Y/Z read consistently between accel and gyro.
AXIS_COLORS = ["#2a78d6", "#eb6834", "#1baf7a"]
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e6e5e0"
REC_RED = "#d03b3b"
DEFAULT_OUT_DIR = Path(__file__).resolve().parent.parent / "recordings"


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
        self.meta = []          # firmware '#' startup lines (chip, bias, ...)
        self.recording = False
        self.rec_lines = []     # raw CSV lines captured while recording

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
                if "LSM6 IMU streamer" in line:  # board was reset: new session
                    self.meta = []
                self.meta.append(line)
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
                if self.recording:
                    self.rec_lines.append(line)  # keep firmware's exact text

    def start_recording(self):
        with self.lock:
            self.rec_lines = []
            self.recording = True

    def stop_recording(self):
        with self.lock:
            self.recording = False
            lines, self.rec_lines = self.rec_lines, []
            return lines, list(self.meta)

    def rec_count(self):
        with self.lock:
            return len(self.rec_lines)

    def snapshot(self):
        with self.lock:
            return list(self.t), [list(c) for c in self.cols]


def safe_name(name):
    name = re.sub(r"[^A-Za-z0-9_-]+", "-", name.strip()).strip("-")
    return name or "test"


def save_recording(out_dir, name, lines, meta, port, started, note=""):
    """Write the CSV and an overview PNG. Returns the CSV path."""
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{started:%Y-%m-%d_%H%M%S}_{safe_name(name)}"
    csv_path = out_dir / f"{stem}.csv"
    rows = [[float(x) for x in ln.split(",")] for ln in lines]
    dur = (rows[-1][0] - rows[0][0]) / 1000.0 if len(rows) > 1 else 0.0
    with open(csv_path, "w") as f:
        f.write(f"# test: {name}\n")
        f.write(f"# started: {started.isoformat(timespec='seconds')}\n")
        f.write(f"# port: {port}\n")
        f.write(f"# samples: {len(rows)}  duration_s: {dur:.2f}  "
                f"rate_hz: {(len(rows) - 1) / dur if dur else 0:.1f}\n")
        if note:
            f.write(f"# note: {note}\n")
        f.write("# units: t_ms = board millis(), a* = g, g* = dps (bias removed), temp_c = degC\n")
        for m in meta:
            f.write(m + "\n")
        f.write(HEADER + "\n")
        for ln in lines:
            f.write(ln + "\n")

    # Overview PNG, drawn off-screen so no extra window pops up.
    if rows:
        t0 = rows[0][0]
        t = [(r[0] - t0) / 1000.0 for r in rows]
        fig = Figure(figsize=(10, 6.5))
        FigureCanvasAgg(fig)
        axs = fig.subplots(2, 1, sharex=True)
        for ax, off, title, names in ((axs[0], 1, "Acceleration (g)", ["ax", "ay", "az"]),
                                      (axs[1], 4, "Angular rate (dps)", ["gx", "gy", "gz"])):
            for k, (n, c) in enumerate(zip(names, AXIS_COLORS)):
                ax.plot(t, [r[off + k] for r in rows], color=c, linewidth=1.4, label=n)
            ax.set_title(title, loc="left", fontsize=11, color=INK)
            ax.grid(True, color=GRID, linewidth=0.8)
            ax.axhline(0, color=MUTED, linewidth=0.8)
            for sp in ("top", "right"):
                ax.spines[sp].set_visible(False)
            ax.legend(loc="upper left", ncol=3, frameon=False, fontsize=9)
        axs[1].set_xlabel("time since start of test (s)")
        fig.suptitle(f"{name}  -  {started:%Y-%m-%d %H:%M:%S}  -  {len(rows)} samples, {dur:.1f} s",
                     x=0.01, ha="left", fontsize=10, color=MUTED)
        fig.tight_layout()
        fig.savefig(csv_path.with_suffix(".png"), dpi=110)
    return csv_path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="serial port (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--window", type=float, default=10.0, help="seconds of history to show")
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR,
                    help=f"where test recordings are saved (default: {DEFAULT_OUT_DIR})")
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
    fig, (ax_a, ax_g) = plt.subplots(2, 1, sharex=True, figsize=(10, 7.2))
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
    fig.tight_layout(rect=(0, 0.11, 1, 1))

    # ---- Test recording controls -----------------------------------------
    fig.text(0.015, 0.045, "Test", fontsize=10, color=INK, va="center")
    name_box = TextBox(fig.add_axes([0.06, 0.025, 0.25, 0.04]), "", initial="test")
    fig.text(0.33, 0.045, "Duration (s)", fontsize=10, color=INK, va="center")
    dur_box = TextBox(fig.add_axes([0.42, 0.025, 0.07, 0.04]), "", initial="")
    rec_btn = Button(fig.add_axes([0.51, 0.02, 0.15, 0.05]), "Start test",
                     color="#eeeeec", hovercolor="#e2e2df")
    rec_msg = fig.text(0.68, 0.045, "blank duration = stop manually", fontsize=9,
                       color=MUTED, va="center")
    rec = {"active": False, "started": None, "t0": 0.0, "dur": None}

    def start_test():
        if not reader.streaming:
            rec_msg.set_text("waiting for data - can't record yet")
            return
        try:
            d = float(dur_box.text) if dur_box.text.strip() else None
            if d is not None and d <= 0:
                raise ValueError
        except ValueError:
            rec_msg.set_text("duration must be a positive number")
            return
        rec.update(active=True, started=dt.datetime.now(), t0=time.monotonic(), dur=d)
        reader.start_recording()
        rec_btn.label.set_text("Stop & save")
        rec_btn.color = rec_btn.hovercolor = "#f6d5d5"
        rec_msg.set_color(REC_RED)

    def stop_test():
        lines, meta = reader.stop_recording()
        rec["active"] = False
        rec_btn.label.set_text("Start test")
        rec_btn.color, rec_btn.hovercolor = "#eeeeec", "#e2e2df"
        rec_msg.set_color(MUTED)
        if not lines:
            rec_msg.set_text("nothing recorded")
            return
        path = save_recording(args.out_dir, name_box.text, lines, meta, port, rec["started"])
        rec_msg.set_text(f"saved {path.name} ({len(lines)} samples)")
        print(f"Saved {path} ({len(lines)} samples) + {path.with_suffix('.png').name}")

    rec_btn.on_clicked(lambda _e: stop_test() if rec["active"] else start_test())

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
        if rec["active"]:
            el = time.monotonic() - rec["t0"]
            if rec["dur"] is not None and el >= rec["dur"]:
                stop_test()
            else:
                left = f" / {rec['dur']:.0f} s" if rec["dur"] else ""
                rec_msg.set_text(f"REC  {el:.1f} s{left}   {reader.rec_count()} samples")
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
        if rec["active"]:  # window closed mid-test: don't lose the data
            stop_test()
        reader.stop = True
        ser.close()


if __name__ == "__main__":
    main()
