#!/usr/bin/env python3
"""Velocity-based analysis of dumbbell biceps-curl sets recorded with the IMU.

Pipeline
  1. Orientation: complementary filter fusing gyro z (integrated) with the
     gravity tilt angle -atan2(ay, ax). A curl is a rotation about the sensor Z
     axis (verified: d/dt tilt == -gz, slope 1.009, r = 0.96).
  2. Side normalisation: the "bottom" of the curl is the extreme closest to
     the resting angle before/after the set; theta_rel = 0 at bottom, +ROM at top.
  3. Rep segmentation: peaks of theta_rel (prominence >= 40 deg). Concentric =
     contiguous interval around the peak's rising phase where omega > 10 dps;
     eccentric = following interval where omega < -10 dps.
  4. Velocity per rep:
       - mean / peak concentric angular velocity (deg/s) straight from the gyro
       - linear velocity two ways:
           a) omega * r  (r = elbow-to-grip radius, default 0.33 m - assumed)
           b) gravity-removed accelerometer, rotated into the vertical plane,
              integrated over the concentric phase with zero velocity at both
              turnarounds (linear drift removal)
  5. RIR: assumes each set ended at momentary failure, so the last full rep
     has RIR = 0 and rep k of N has RIR = N - k. Four estimators are compared
     with leave-one-trial-out cross-validation.

Usage
  .venv/bin/python analysis/analyze_curls.py           # uses the trial map below
Outputs go to analysis/output/.
"""
from __future__ import annotations

import glob
import json
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt, find_peaks

ROOT = Path(__file__).resolve().parent.parent
REC = ROOT / "recordings"
OUT = ROOT / "analysis" / "output"

# Trial map (test2 was incomplete and is replaced by test2correct).
TRIALS = [
    ("test1", "L", 1),
    ("test2correct", "R", 1),
    ("test3", "L", 2),
    ("test4", "R", 2),
    ("test5", "L", 3),
    ("test6", "R", 3),
]

FOREARM_R = 0.33        # m, elbow axis -> dumbbell grip (assumption, adjustable)
OMEGA_THR = 10.0        # dps, motion threshold for phase boundaries
MIN_PROM = 40.0         # deg, prominence used to find the bottom/top extremes
UP_DEG = 40.0           # deg above bottom that starts a lift attempt
DOWN_DEG = 25.0         # deg above bottom that ends it (hysteresis)
TOP_TOL = 3.0           # deg, "at the top" tolerance
FULL_ROM_FRAC = 0.70    # rep counts as "full" if ROM >= 70% of the set's median ROM
G = 9.80665

# Validated categorical palette (slot 1 blue = Left, slot 2 orange = Right)
C_L, C_R, C_3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e6e5e0"
SIDE_C = {"L": C_L, "R": C_R}
SIDE_NAME = {"L": "Left", "R": "Right"}
plt.rcParams.update({
    "axes.edgecolor": GRID, "axes.labelcolor": MUTED, "xtick.color": MUTED,
    "ytick.color": MUTED, "text.color": INK, "axes.titlecolor": INK,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
    "grid.color": GRID, "grid.linewidth": 0.8, "font.size": 10,
    "axes.titlesize": 11, "axes.titlelocation": "left", "legend.frameon": False,
})


# --------------------------------------------------------------------------- #
def load(name: str) -> pd.DataFrame:
    path = next(iter(glob.glob(str(REC / f"*_{name}.csv"))))
    d = pd.read_csv(path, comment="#")
    d["t"] = (d.t_ms - d.t_ms.iloc[0]) / 1000.0
    return d


def lowpass(x, fs, fc, order=4):
    b, a = butter(order, fc / (fs / 2))
    return filtfilt(b, a, x)


def complementary(gz_dps, theta_acc_deg, dt, tau=1.0):
    """Gyro-dominant tilt about Z; accelerometer only corrects slow drift."""
    alpha = tau / (tau + dt)
    th = np.empty_like(theta_acc_deg)
    th[0] = theta_acc_deg[0]
    for k in range(1, len(th)):
        th[k] = alpha * (th[k - 1] + gz_dps[k] * dt) + (1 - alpha) * theta_acc_deg[k]
    return th


@dataclass
class Rep:
    idx: int
    i0: int            # concentric start
    i_pk: int          # top
    i1: int            # eccentric end
    rom: float         # deg
    t_con: float       # s
    t_ecc: float       # s
    mcv_ang: float     # deg/s, mean concentric angular velocity
    pcv_ang: float     # deg/s, peak concentric angular velocity
    mcv_lin_r: float   # m/s, mean concentric velocity = omega * r
    mcv_lin_acc: float  # m/s, mean concentric speed from accelerometer integration
    mcv_vert_acc: float  # m/s, mean concentric vertical velocity (accel)
    disp_vert: float   # m, vertical displacement during concentric (accel)
    stall_s: float = 0.0  # s with |omega| < threshold inside the concentric (sticking)
    kind: str = "full"    # full | partial | failed
    full: bool = True
    rir: float = np.nan


@dataclass
class Trial:
    name: str
    side: str
    set_no: int
    d: pd.DataFrame
    fs: float = 0.0
    theta_rel: np.ndarray = None
    omega: np.ndarray = None
    sign: int = 1
    reps: list = field(default_factory=list)
    v_vert: np.ndarray = None
    phase_check: str = ""
    failed_attempt: bool = False

    @property
    def full_reps(self):
        return [r for r in self.reps if r.full]


# --------------------------------------------------------------------------- #
def analyse(name, side, set_no) -> Trial:
    d = load(name)
    tr = Trial(name, side, set_no, d)
    t = d.t.to_numpy()
    fs = 1 / np.median(np.diff(t))
    tr.fs = fs
    dt = 1 / fs

    ax, ay = lowpass(d.ax, fs, 8), lowpass(d.ay, fs, 8)
    gz = lowpass(d.gz, fs, 8)
    theta_acc = np.degrees(-np.unwrap(np.arctan2(ay, ax)))
    theta = complementary(gz, theta_acc, dt)

    # --- find the oscillation extremes on a smoothed angle -----------------
    th_s = lowpass(theta, fs, 2)
    hi, _ = find_peaks(th_s, prominence=MIN_PROM, distance=int(0.8 * fs))
    lo, _ = find_peaks(-th_s, prominence=MIN_PROM, distance=int(0.8 * fs))
    hi_med, lo_med = np.median(th_s[hi]), np.median(th_s[lo])
    # Rest angle: median of first and last 1 s
    n1 = int(fs)
    rest = np.median(np.r_[th_s[:n1], th_s[-n1:]])
    tr.sign = 1 if abs(rest - lo_med) <= abs(rest - hi_med) else -1
    bottom = lo_med if tr.sign == 1 else hi_med
    tr.theta_rel = tr.sign * (theta - bottom)
    tr.omega = tr.sign * gz

    # --- linear acceleration in the vertical plane --------------------------
    thr = np.radians(theta)
    u = np.stack([np.cos(thr), -np.sin(thr)], 1)       # world "up" in sensor x,y
    h = np.stack([np.sin(thr), np.cos(thr)], 1)        # world horizontal in plane
    f = np.stack([ax, ay], 1)
    a_lin = f - u                                      # g
    a_v = (a_lin * u).sum(1) * G                       # m/s^2
    a_h = (a_lin * h).sum(1) * G
    tr.v_vert = np.full(len(t), np.nan)

    # --- reps: excursions of the angle away from the bottom ----------------
    # A lift attempt starts when theta rises through UP_DEG and ends when it
    # falls back below DOWN_DEG (hysteresis). The concentric phase runs from
    # motion onset to the top, *including* any sticking-point pause - splitting
    # at the pause would cut the hardest reps of the set in two.
    th_r = lowpass(tr.theta_rel, fs, 2)
    om = tr.omega
    n = len(t)
    attempts, i, up = [], 0, False
    for k in range(n):
        if not up and th_r[k] > UP_DEG:
            up, k_up = True, k
        elif up and th_r[k] < DOWN_DEG:
            attempts.append((k_up, k))
            up = False
    if up:
        attempts.append((k_up, n - 1))

    reps = []
    prev_end = 0
    for k_up, k_dn in attempts:
        i_peak = k_up + int(np.argmax(th_r[k_up:k_dn + 1]))
        peak = th_r[i_peak]
        # onset: walk back from the up-crossing until motion stops
        i0 = k_up
        while i0 > prev_end and om[i0] > OMEGA_THR:
            i0 -= 1
        # concentric end: first time within TOP_TOL of the peak
        i_top = k_up + int(np.argmax(th_r[k_up:i_peak + 1] >= peak - TOP_TOL))
        # eccentric: last time near the peak, until motion stops after the down-crossing
        after = th_r[i_peak:k_dn + 1] >= peak - TOP_TOL
        ecc0 = i_peak + int(len(after) - 1 - np.argmax(after[::-1]))
        i1 = k_dn
        while i1 < n - 1 and om[i1] < -OMEGA_THR:
            i1 += 1
        prev_end = i1

        rom = th_r[i_top] - th_r[i0]
        t_con = t[i_top] - t[i0]
        t_ecc = t[i1] - t[ecc0]
        if t_con <= 0:
            continue
        w = om[i0:i_top + 1]
        # time spent stalled (|omega| < threshold) inside the concentric phase
        stall = float(np.mean(np.abs(w) < OMEGA_THR) * t_con)

        sl = slice(i0, i_top + 1)
        vv = np.cumsum(a_v[sl]) * dt
        vh = np.cumsum(a_h[sl]) * dt
        ramp = np.linspace(0, 1, len(vv))
        vv -= ramp * vv[-1]
        vh -= ramp * vh[-1]
        tr.v_vert[sl] = vv
        speed = np.hypot(vv, vh)

        reps.append(Rep(
            idx=0, i0=i0, i_pk=i_top, i1=i1, rom=rom, t_con=t_con, t_ecc=t_ecc,
            mcv_ang=rom / t_con, pcv_ang=float(w.max()),
            mcv_lin_r=np.radians(rom / t_con) * FOREARM_R,
            mcv_lin_acc=float(speed.mean()), mcv_vert_acc=float(vv.mean()),
            disp_vert=float(np.sum(vv) * dt), stall_s=stall,
        ))

    med = np.median([r.rom for r in reps]) if reps else 0
    k = 0
    for r in reps:
        r.full = r.rom >= FULL_ROM_FRAC * med
        if r.full:
            k += 1
            r.idx = k
    n_full = k
    last_full_i = max((j for j, r in enumerate(reps) if r.full), default=-1)
    for j, r in enumerate(reps):
        if r.full:
            r.rir = n_full - r.idx
        elif j > last_full_i and (r.t_con >= 1.5 or r.stall_s >= 0.3):
            r.kind = "failed"      # slow/stalled attempt after the last full rep
        elif j > last_full_i:
            r.kind = "handling"    # quick move after the set (re-racking, putting down)
        else:
            r.kind = "partial"
    tr.failed_attempt = any(r.kind == "failed" for r in reps)
    tr.reps = reps

    # direction sanity check: concentric (lifting) usually faster than eccentric
    fr = tr.full_reps
    if fr:
        tc, te = np.median([r.t_con for r in fr]), np.median([r.t_ecc for r in fr])
        vz = np.median([r.mcv_vert_acc for r in fr])
        tr.phase_check = (f"median concentric {tc:.2f} s vs eccentric {te:.2f} s; "
                          f"median vertical velocity during 'concentric' {vz:+.2f} m/s "
                          f"({'up - direction confirmed' if vz > 0 else 'DOWN - check sign'})")
    return tr


# --------------------------------------------------------------------------- #
def rep_table(trials):
    rows = []
    for tr in trials:
        for r in tr.reps:
            rows.append(dict(
                trial=tr.name, side=tr.side, set=tr.set_no, rep=r.idx if r.full else None,
                full=r.full, kind=r.kind, rir=r.rir, t_start=round(tr.d.t.iloc[r.i0], 2),
                t_top=round(tr.d.t.iloc[r.i_pk], 2), t_end=round(tr.d.t.iloc[r.i1], 2),
                rom_deg=round(r.rom, 1), t_con_s=round(r.t_con, 3), t_ecc_s=round(r.t_ecc, 3),
                mcv_ang_dps=round(r.mcv_ang, 1), pcv_ang_dps=round(r.pcv_ang, 1),
                mcv_lin_r_ms=round(r.mcv_lin_r, 3), mcv_speed_acc_ms=round(r.mcv_lin_acc, 3),
                mcv_vert_acc_ms=round(r.mcv_vert_acc, 3), disp_vert_m=round(r.disp_vert, 3),
                stall_s=round(r.stall_s, 2),
            ))
    df = pd.DataFrame(rows)
    # Velocity loss relative to the fastest rep *so far* in the set (causal, so
    # it can be computed live). Reps 1-2 are often slower than 3-5 (warm-up).
    df["vl_pct"] = np.nan
    for tr in trials:
        m = (df.trial == tr.name) & df.full
        g = df.loc[m].sort_values("rep")
        ref = g.mcv_ang_dps.cummax()
        df.loc[g.index, "vl_pct"] = (100 * (1 - g.mcv_ang_dps / ref)).round(1)
    return df


# --------------------------------------------------------------------------- #
def rir_models(df):
    """Leave-one-trial-out evaluation of RIR estimators."""
    full = df[df.full].copy()
    names = full.trial.unique()
    preds = []
    for test in names:
        tr_ = full[full.trial != test]
        te = full[full.trial == test].sort_values("rep")
        N_train = tr_.groupby("trial").rep.max()
        last_mcv = tr_.loc[tr_.groupby("trial").rep.idxmax(), "mcv_ang_dps"]
        mvt = last_mcv.mean()

        # B: RIR ~ velocity loss;  C: RIR ~ absolute MCV
        bB = np.polyfit(tr_.vl_pct, tr_.rir, 1)
        bC = np.polyfit(tr_.mcv_ang_dps, tr_.rir, 1)
        # E: RIR ~ rep number + velocity loss (does velocity add to counting?)
        XE = np.c_[np.ones(len(tr_)), tr_.rep, tr_.vl_pct]
        bE = np.linalg.lstsq(XE, tr_.rir.to_numpy(), rcond=None)[0]

        mcvs = te.mcv_ang_dps.to_numpy()
        for row in te.itertuples():
            k = int(row.rep)
            base = max(0.0, N_train.mean() - k)
            pB = max(0.0, np.polyval(bB, row.vl_pct))
            pC = max(0.0, np.polyval(bC, row.mcv_ang_dps))
            pE = max(0.0, float(bE @ [1, k, row.vl_pct]))
            # D: linear trend of MCV from the fastest rep so far, extrapolated to
            # the minimum velocity threshold (mean last-rep MCV of other sets).
            pD = base
            k_best = int(np.argmax(mcvs[:k]))
            if k - k_best >= 3:
                kk = np.arange(k_best + 1, k + 1)
                sl, c = np.polyfit(kk, mcvs[k_best:k], 1)
                if sl < 0:
                    pD = float(np.clip((mvt - c) / sl - k, 0, 15))
            preds.append(dict(trial=test, side=row.side, rep=k, rir=row.rir,
                              A_baseline=base, B_vel_loss=pB, C_abs_mcv=pC,
                              D_mvt_extrap=pD, E_rep_plus_vl=pE, mvt=mvt))
    p = pd.DataFrame(preds)
    models = ["A_baseline", "B_vel_loss", "C_abs_mcv", "D_mvt_extrap", "E_rep_plus_vl"]
    summ = []
    for m in models:
        e = p[m] - p.rir
        late = p.rir <= 3
        summ.append(dict(model=m, MAE_all=round(e.abs().mean(), 2),
                         MAE_RIR_le3=round(e[late].abs().mean(), 2),
                         within_1_rep_pct=round(100 * (e.abs() <= 1).mean(), 0),
                         bias=round(e.mean(), 2)))
    # Full-data fits for reporting
    fit = dict(
        vel_loss=np.polyfit(full.vl_pct, full.rir, 1).tolist(),
        abs_mcv=np.polyfit(full.mcv_ang_dps, full.rir, 1).tolist(),
        r_vl=float(np.corrcoef(full.vl_pct, full.rir)[0, 1]),
        r_mcv=float(np.corrcoef(full.mcv_ang_dps, full.rir)[0, 1]),
        r_tcon=float(np.corrcoef(full.t_con_s, full.rir)[0, 1]),
        mvt_all=float(full.loc[full.groupby("trial").rep.idxmax(), "mcv_ang_dps"].mean()),
    )
    return p, pd.DataFrame(summ), fit


# --------------------------------------------------------------------------- #
def plot_trial(tr: Trial, path):
    t = tr.d.t.to_numpy()
    c = SIDE_C[tr.side]
    fig, axs = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True,
                            gridspec_kw=dict(height_ratios=[1.2, 1, 0.8]))
    fig.suptitle(f"{tr.name} - {SIDE_NAME[tr.side]} arm, set {tr.set_no} - "
                 f"{len(tr.full_reps)} full reps", x=0.01, ha="left", fontsize=12)
    axs[0].plot(t, tr.theta_rel, color=c, lw=1.6)
    axs[0].set_title("Dumbbell / forearm angle from bottom (deg)")
    axs[1].plot(t, tr.omega, color=c, lw=1.4)
    axs[1].axhline(0, color=MUTED, lw=0.8)
    axs[1].set_title("Angular velocity, + = lifting (deg/s)")
    axs[2].plot(t, tr.v_vert, color=c, lw=1.4)
    axs[2].axhline(0, color=MUTED, lw=0.8)
    axs[2].set_title("Vertical velocity during concentric, from accelerometer (m/s)")
    axs[2].set_xlabel("time (s)")
    for r in tr.reps:
        for ax in axs:
            ax.axvspan(t[r.i0], t[r.i_pk], color=c if r.full else MUTED,
                       alpha=0.12 if r.full else 0.18, lw=0)
        lab = str(r.idx) if r.full else r.kind
        axs[0].annotate(lab, (t[r.i_pk], tr.theta_rel[r.i_pk]), xytext=(0, 4),
                        textcoords="offset points", ha="center", fontsize=9, color=INK)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_velocity_by_rep(df, path):
    full = df[df.full]
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.6))
    markers = {1: "o", 2: "s", 3: "^"}
    for (trial, side, st), g in full.groupby(["trial", "side", "set"], sort=False):
        g = g.sort_values("rep")
        kw = dict(color=SIDE_C[side], marker=markers[st], ms=6, lw=1.8,
                  label=f"{SIDE_NAME[side]} set {st}")
        axs[0].plot(g.rep, g.mcv_ang_dps, **kw)
        axs[1].plot(g.rep, g.vl_pct, **kw)
        axs[2].plot(g.rir, g.mcv_ang_dps, **kw)
    axs[0].set(title="Mean concentric angular velocity by rep", xlabel="rep", ylabel="deg/s")
    axs[1].set(title="Velocity loss vs fastest rep so far", xlabel="rep", ylabel="%")
    axs[2].set(title="Velocity vs reps in reserve (last rep = 0)", xlabel="RIR", ylabel="deg/s")
    axs[2].invert_xaxis()
    for ax in axs:
        ax.xaxis.get_major_locator().set_params(integer=True)
    axs[0].legend(ncol=2, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def plot_rir(p, fit, path):
    ms = ["A_baseline", "B_vel_loss", "C_abs_mcv", "E_rep_plus_vl"]
    fig, axs = plt.subplots(1, 4, figsize=(18, 4.6), sharey=True)
    lim = max(p.rir.max(), p[ms].max().max()) + 1
    for ax, m, title in zip(axs, ms,
                            ["A: rep count only", "B: velocity loss %",
                             "C: absolute velocity", "E: rep count + velocity loss"]):
        for side in "LR":
            q = p[p.side == side]
            ax.scatter(q.rir, q[m], s=36, color=SIDE_C[side], label=SIDE_NAME[side],
                       edgecolor="white", lw=1.2, zorder=3)
        ax.plot([0, lim], [0, lim], color=MUTED, lw=1, ls="--")
        ax.fill_between([0, lim], [-1, lim - 1], [1, lim + 1], color=MUTED, alpha=0.08, lw=0)
        e = (p[m] - p.rir).abs().mean()
        ax.set(title=f"{title}  (MAE {e:.1f} reps)", xlabel="actual RIR", xlim=(-0.5, lim),
               ylim=(-0.5, lim))
    axs[0].set_ylabel("predicted RIR (leave-one-set-out)")
    axs[0].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# --------------------------------------------------------------------------- #
def main():
    OUT.mkdir(parents=True, exist_ok=True)
    trials = [analyse(*x) for x in TRIALS]
    for tr in trials:
        plot_trial(tr, OUT / f"trial_{tr.name}.png")
    df = rep_table(trials)
    df.to_csv(OUT / "reps.csv", index=False)
    plot_velocity_by_rep(df, OUT / "velocity_by_rep.png")
    p, summ, fit = rir_models(df)
    p.round(2).to_csv(OUT / "rir_predictions.csv", index=False)
    summ.to_csv(OUT / "rir_model_summary.csv", index=False)
    plot_rir(p, fit, OUT / "rir_predictions.png")

    pd.set_option("display.width", 200, "display.max_columns", 30, "display.max_rows", 200)
    print("== Per-trial summary")
    for tr in trials:
        fr = tr.full_reps
        g = df[(df.trial == tr.name) & df.full]
        kinds = pd.Series([r.kind for r in tr.reps if not r.full]).value_counts().to_dict()
        extra = ", ".join(f"{v} {k}" for k, v in kinds.items())
        print(f"{tr.name:13s} {tr.side} set{tr.set_no}: {len(fr):2d} full reps"
              f"{f' (+{extra})' if extra else ''} | ROM {g.rom_deg.median():.0f} deg | "
              f"MCV first2 {g.mcv_ang_dps.iloc[:2].mean():.0f} -> last {g.mcv_ang_dps.iloc[-1]:.0f} dps "
              f"(VL {g.vl_pct.iloc[-1]:.0f}%) | t_con {g.t_con_s.iloc[:2].mean():.2f} -> "
              f"{g.t_con_s.iloc[-1]:.2f} s")
        print(f"{'':16s}{tr.phase_check}")
        print(f"{'':16s}ended with failed attempt: {tr.failed_attempt}; "
              f"sticking time on last full rep: {g.stall_s.iloc[-1]:.2f} s")
    print("\n== Reps")
    print(df.to_string(index=False))
    print("\n== RIR models (leave-one-set-out)")
    print(summ.to_string(index=False))
    print("\n== Fits on all data")
    print(json.dumps(fit, indent=1))


if __name__ == "__main__":
    main()
