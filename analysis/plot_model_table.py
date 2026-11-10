#!/usr/bin/env python3
"""One-figure summary: what each RIR model uses, and how well it performed.

Recomputes the trained-on (in-sample) error for each model and combines it with
the leave-one-set-out results from model_comparison.py.
Output: analysis/output/model_table.png
"""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
import model_comparison as mc  # noqa: E402

OUT = mc.OUT
BLUE, ORANGE = "#2a78d6", "#eb6834"
INK, MUTED, GRID, SURFACE = "#1f1f1e", "#6b6a64", "#e6e5e0", "#fcfcfb"

V, R, E = ["mcv2", "vl"], ["rest_k", "rest2"], ["tcon", "stall", "pcv"]
MODELS = [
    # label, inputs, combination, key in model_comparison.csv, fit function
    ("Current", "speed, velocity loss", "weighted sum",
     "current: velocity (linear)", mc.linear(V)),
    ("+ rest before rep", "speed, velocity loss, rest", "weighted sum",
     "velocity + rest before rep (linear)", mc.linear(V + ["rest_k"])),
    ("+ avg rest of last 2 reps", "speed, velocity loss, rest", "weighted sum",
     "velocity + avg rest last 2 (linear)", mc.linear(V + ["rest2"])),
    ("Ridge", "speed, vel. loss, rest, effort cues", "weighted sum, penalised",
     "velocity + rest + effort (ridge)", mc.ridge(V + R + E)),
    ("Random forest", "same 7 inputs", "average of 400 trees",
     "velocity + rest + effort (random forest)", mc.forest(V + R + E)),
    ("Gradient boosting", "same 7 inputs", "150 trees, each fixing the last",
     "velocity + rest + effort (boosting)", mc.boosting(V + R + E)),
    ("Fatigue-recovery", "speed, vel. loss, fatigue score", "weighted sum",
     "velocity + fatigue-recovery state F", mc.fatigue_model([])),
]


def main():
    d = mc.build()
    res = pd.read_csv(OUT / "model_comparison.csv").set_index("model")
    rows = []
    for label, inputs, how, key, fp in MODELS:
        ins = np.abs(np.clip(fp(d, d), 0, None) - d.rir).mean()
        r = res.loc[key]
        rows.append(dict(label=label, inputs=inputs, how=how, held=r.MAE, ins=ins,
                         near=r.MAE_rir_0_2, within1=r.within1,
                         better=int(r.sets_better), worse=int(r.sets_worse)))
    t = pd.DataFrame(rows)
    t.round(3).to_csv(OUT / "model_table.csv", index=False)
    print(t.round(2).to_string(index=False))

    n = len(t)
    fig = plt.figure(figsize=(16, 6.4), facecolor=SURFACE)
    # column x-positions (figure fraction) for the text part
    cols = [(0.010, "Model"), (0.135, "Inputs"), (0.300, "How they're combined")]
    num_cols = [(0.770, "Near failure\n(RIR 0–2)"), (0.845, "Within\n±1 rep"),
                (0.910, "Sets better/\nworse vs current")]
    top, bot = 0.79, 0.24
    ys = np.linspace(top, bot, n)
    row_h = (top - bot) / (n - 1)

    fig.text(0.010, 0.955, "RIR models for stopped-early sets: what each uses and how it performed",
             fontsize=13, color=INK, weight="bold")
    fig.text(0.010, 0.915, "Leave-one-set-out: trained on 5 sets, tested on the 6th, set stopped at "
             "every rep from 3 onward (45 predictions). Lower error is better.",
             fontsize=9.5, color=MUTED)
    for x, h in cols + num_cols:
        fig.text(x, 0.865, h, fontsize=9.5, color=MUTED, va="center", weight="bold")
    best = t.held.idxmin()
    # stripes on a background layer, drawn before (= behind) the chart axes
    bg = fig.add_axes([0, 0, 1, 1], zorder=-1)
    bg.set_xlim(0, 1)
    bg.set_ylim(0, 1)
    bg.axis("off")
    for i, y in enumerate(ys):
        if i % 2 == 0:
            bg.axhspan(y - row_h / 2, y + row_h / 2, xmin=0.005, xmax=0.995,
                       color="#f3f2ee", lw=0)
    for i, (y, r) in enumerate(zip(ys, t.itertuples())):
        w = "bold" if i in (0, best) else "normal"
        fig.text(cols[0][0], y, r.label, fontsize=10, color=INK, va="center", weight=w)
        fig.text(cols[1][0], y, r.inputs, fontsize=9.5, color=INK, va="center")
        fig.text(cols[2][0], y, r.how, fontsize=9.5, color=INK, va="center")
        fig.text(num_cols[0][0] + 0.02, y, f"{r.near:.2f}", fontsize=10, color=INK,
                 va="center", ha="center")
        fig.text(num_cols[1][0] + 0.02, y, f"{r.within1:.0f}%", fontsize=10, color=INK,
                 va="center", ha="center")
        fig.text(num_cols[2][0] + 0.035, y,
                 "—" if i == 0 else f"{r.better} / {r.worse}", fontsize=10, color=INK,
                 va="center", ha="center")

    # error chart, rows aligned with the table
    ax = fig.add_axes([0.475, bot - row_h / 2, 0.27, top - bot + row_h])
    ax.set_facecolor("none")
    yy = np.arange(n)[::-1]
    for y, r in zip(yy, t.itertuples()):
        ax.plot([r.ins, r.held], [y, y], color=GRID, lw=2.5, zorder=1, solid_capstyle="round")
    ax.scatter(t.ins, yy, s=70, facecolor=SURFACE, edgecolor=MUTED, lw=1.8, zorder=2,
               label="on the 5 sets it was trained on")
    ax.scatter(t.held, yy, s=80, color=[ORANGE if i == best else BLUE for i in range(n)],
               edgecolor=SURFACE, lw=1.5, zorder=3, label="on the unseen 6th set")
    for y, r in zip(yy, t.itertuples()):
        ax.text(r.held + 0.035, y, f"{r.held:.2f}", va="center", fontsize=10, color=INK,
                weight="bold")
    ax.axvline(t.held.iloc[0], color=MUTED, ls="--", lw=1, zorder=0)
    ax.set_xlim(0, 1.15)
    ax.set_ylim(-0.5, n - 0.5)
    ax.set_yticks([])
    ax.set_xlabel("average error (reps)", color=MUTED, fontsize=9.5)
    ax.tick_params(colors=MUTED, labelsize=9)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color(GRID)
    ax.grid(axis="x", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    fig.text(0.475, 0.865, "Average error (reps)", fontsize=9.5, color=MUTED,
             va="center", weight="bold")
    fig.text(0.475, 0.838, "○ on the 5 sets it trained on   ● on the unseen 6th set   "
             "gap = overfitting", fontsize=8.5, color=MUTED, va="center")


    fig.text(0.010, 0.025,
             "Fatigue-recovery's lead mostly comes from its fatigue score tracking reps done, so it "
             "learns this weight's set length (~10 reps) and is unlikely to transfer to other "
             "weights. Speed = 2-rep average lifting speed (°/s).",
             fontsize=8.5, color=MUTED)
    fig.savefig(OUT / "model_table.png", dpi=130, facecolor=SURFACE)
    plt.close(fig)


if __name__ == "__main__":
    main()
