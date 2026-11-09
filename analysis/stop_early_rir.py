#!/usr/bin/env python3
"""Estimate RIR for a set stopped before failure, using velocity only.

Simulation: every to-failure set is truncated at each rep k; the model sees
only reps 1..k (no knowledge of how long sets usually last at this load) and
predicts RIR = N - k. Leave-one-set-out: models are fit on 5 sets, tested on
the 6th.

Features available at the stopping rep k (all computable live):
  mcv     mean concentric velocity of rep k (deg/s)
  mcv2    mean of reps k-1 and k (less noise)
  vl      velocity loss of rep k vs fastest rep so far (%)
  vl2     same, using the 2-rep mean
  tcon    concentric duration of rep k (s)

Outputs analysis/output/stop_early_*.csv/png.
"""
from itertools import combinations
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

OUT = Path(__file__).resolve().parent / "output"
C_L, C_R, C_3 = "#2a78d6", "#eb6834", "#1baf7a"
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e6e5e0"
plt.rcParams.update({
    "axes.edgecolor": GRID, "axes.labelcolor": MUTED, "xtick.color": MUTED,
    "ytick.color": MUTED, "text.color": INK, "axes.spines.top": False,
    "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
    "axes.titlelocation": "left", "axes.titlesize": 11, "legend.frameon": False,
})


def features():
    df = pd.read_csv(OUT / "reps.csv")
    df = df[df.full].copy()
    rows = []
    for name, g in df.groupby("trial", sort=False):
        g = g.sort_values("rep")
        v = g.mcv_ang_dps.to_numpy()
        v2 = pd.Series(v).rolling(2, min_periods=1).mean().to_numpy()
        runmax = np.maximum.accumulate(v)
        runmax2 = np.maximum.accumulate(v2)
        for j in range(len(g)):
            rows.append(dict(trial=name, side=g.side.iloc[0], k=j + 1, rir=len(g) - 1 - j,
                             mcv=v[j], mcv2=v2[j],
                             vl=100 * (1 - v[j] / runmax[j]),
                             vl2=100 * (1 - v2[j] / runmax2[j]),
                             tcon=g.t_con_s.iloc[j]))
    return pd.DataFrame(rows)


def loso(f, feats):
    pred = np.empty(len(f))
    for t in f.trial.unique():
        tr, te = f.trial != t, f.trial == t
        X = np.c_[np.ones(tr.sum()), f.loc[tr, feats]]
        b = np.linalg.lstsq(X, f.loc[tr, "rir"], rcond=None)[0]
        pred[te.to_numpy()] = np.c_[np.ones(te.sum()), f.loc[te, feats]] @ b
    return np.clip(pred, 0, None)


def main():
    f = features()
    f = f[f.k >= 3].reset_index(drop=True)   # need a few reps to define "fastest so far"

    cand = ["mcv", "mcv2", "vl", "vl2", "tcon"]
    res = []
    for r in (1, 2):
        for feats in combinations(cand, r):
            if {"mcv", "mcv2"} <= set(feats) or {"vl", "vl2"} <= set(feats):
                continue
            p = loso(f, list(feats))
            e = np.abs(p - f.rir)
            res.append(dict(features="+".join(feats), MAE=e.mean(),
                            MAE_rir_0_2=e[f.rir <= 2].mean(),
                            MAE_rir_3_5=e[(f.rir >= 3) & (f.rir <= 5)].mean(),
                            MAE_rir_6p=e[f.rir >= 6].mean(),
                            within1=100 * (e <= 1).mean()))
    res = pd.DataFrame(res).sort_values("MAE")
    print("Velocity-only RIR models, sets truncated at every rep k>=3 (leave-one-set-out):")
    print(res.round(2).to_string(index=False))
    res.round(3).to_csv(OUT / "stop_early_models.csv", index=False)

    best = res.iloc[0].features.split("+")
    f["pred"] = loso(f, best)
    f["err"] = f.pred - f.rir
    f.round(2).to_csv(OUT / "stop_early_predictions.csv", index=False)

    # ---- practical question: am I within 2 reps of failure? ----------------
    print("\n'Within 2 reps of failure?' (RIR <= 2), leave-one-set-out threshold:")
    for feat, direction in [("vl2", ">"), ("vl", ">"), ("mcv2", "<"), ("mcv", "<")]:
        correct, thr_used = [], []
        for t in f.trial.unique():
            tr, te = f[f.trial != t], f[f.trial == t]
            y = tr.rir <= 2
            grid = np.unique(tr[feat])
            accs = [((tr[feat] > g) if direction == ">" else (tr[feat] < g)).eq(y).mean()
                    for g in grid]
            g = grid[int(np.argmax(accs))]
            thr_used.append(g)
            yhat = (te[feat] > g) if direction == ">" else (te[feat] < g)
            correct.extend((yhat == (te.rir <= 2)).tolist())
        print(f"  {feat:5s} {direction} ~{np.median(thr_used):5.1f}: "
              f"{100 * np.mean(correct):.0f}% correct")
    # sensitivity / specificity of the speed rule, threshold fit on all sets
    for thr in (85, 90, 95):
        hit = f[f.rir <= 2].mcv2.lt(thr).mean()
        fa = f[f.rir > 2].mcv2.lt(thr).mean()
        print(f"  rule '2-rep avg MCV < {thr} deg/s': catches {100 * hit:.0f}% of RIR<=2 reps, "
              f"false alarm on {100 * fa:.0f}% of RIR>=3 reps")
    # same subset, count prior for reference (knows typical set length)
    cp = []
    for t in f.trial.unique():
        n_other = f[f.trial != t].groupby("trial").k.max().mean()
        te = f[f.trial == t]
        cp.extend(np.abs(np.clip(n_other - te.k, 0, None) - te.rir).tolist())
    print(f"\nReference - count prior on the same reps (needs known set length): MAE {np.mean(cp):.2f}")

    # ---- error by true RIR for the best model -----------------------------
    by = f.groupby("rir").agg(n=("err", "size"), mean_pred=("pred", "mean"),
                              mae=("err", lambda e: e.abs().mean()))
    print(f"\nBest model ({'+'.join(best)}): prediction by true RIR")
    print(by.round(2).to_string())

    # ---- figure ------------------------------------------------------------
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.6))
    for side, c in (("L", C_L), ("R", C_R)):
        q = f[f.side == side]
        jit = (np.random.default_rng(0).random(len(q)) - 0.5) * 0.25
        axs[0].scatter(q.rir + jit, q.pred, color=c, s=34, edgecolor="white", lw=1.2,
                       label="Left" if side == "L" else "Right", zorder=3)
    lim = f.rir.max() + 1
    axs[0].plot([0, lim], [0, lim], color=MUTED, ls="--", lw=1)
    axs[0].fill_between([0, lim], [-1, lim - 1], [1, lim + 1], color=MUTED, alpha=0.08, lw=0)
    axs[0].set(title=f"Stopped-early sets: velocity-only estimate ({'+'.join(best)})",
               xlabel="true RIR when the set was stopped", ylabel="estimated RIR",
               xlim=(-0.5, lim), ylim=(-0.5, lim))
    axs[0].legend()
    axs[1].bar(by.index, by.mae, color=C_3, width=0.6)
    axs[1].set(title="Error by true RIR", xlabel="true RIR", ylabel="mean abs. error (reps)")
    axs[1].xaxis.get_major_locator().set_params(integer=True)
    fig.tight_layout()
    fig.savefig(OUT / "stop_early_rir.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
