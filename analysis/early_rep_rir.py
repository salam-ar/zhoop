#!/usr/bin/env python3
"""Can the velocity decline over the first k reps predict reps in reserve?

Reads analysis/output/reps.csv (run analyze_curls.py first).

1. Shape consistency: each set's mean concentric velocity (MCV) is normalised
   to its own reps 1-3 average, then the curves are compared aligned by rep
   number (from the start) and aligned by RIR (from failure).
2. Early prediction, leave-one-set-out, after observing reps 1..k:
   A  count prior       RIR = mean(N of other sets) - k
   B  velocity loss     RIR = a + b * VL_k             (fit on other sets, all reps)
   S  slope             RIR = a + b * slope(reps 1..k) (fit on other sets at the same k)
   T  template match    slide the average "decline-to-failure" curve of the
                        other sets along the observed reps; the best-fitting
                        failure point N gives RIR = N - k
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

OUT = Path(__file__).resolve().parent / "output"
C_L, C_R = "#2a78d6", "#eb6834"
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e6e5e0"
SIDE_C = {"L": C_L, "R": C_R}
plt.rcParams.update({
    "axes.edgecolor": GRID, "axes.labelcolor": MUTED, "xtick.color": MUTED,
    "ytick.color": MUTED, "text.color": INK, "axes.spines.top": False,
    "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
    "axes.titlelocation": "left", "axes.titlesize": 11, "legend.frameon": False,
})
MAX_RIR_T = 8   # template defined for RIR 0..8 (every set has >= 9 reps)


def load():
    df = pd.read_csv(OUT / "reps.csv")
    df = df[df.full].copy()
    df["rep"] = df.rep.astype(int)
    sets = {}
    for name, g in df.groupby("trial", sort=False):
        g = g.sort_values("rep")
        v = g.mcv_ang_dps.to_numpy()
        sets[name] = dict(side=g.side.iloc[0], set=int(g.set.iloc[0]), v=v,
                          vn=v / v[:3].mean(), N=len(v))
    return df, sets


def template(sets, names):
    """Mean normalised velocity as a function of RIR (0..MAX_RIR_T)."""
    rows = []
    for n in names:
        s = sets[n]
        for j, val in enumerate(s["vn"]):
            r = s["N"] - 1 - j
            if r <= MAX_RIR_T:
                rows.append((r, val))
    t = pd.DataFrame(rows, columns=["rir", "v"]).groupby("rir").v.mean()
    return t.reindex(range(MAX_RIR_T + 1)).to_numpy()


def template_predict(vn_obs, T, k, n_max=20):
    """Best failure rep N >= k for the first k normalised velocities."""
    best, best_err = k, np.inf
    for N in range(k, n_max + 1):
        rirs = N - 1 - np.arange(k)              # RIR of reps 1..k
        pred = T[np.clip(rirs, 0, MAX_RIR_T)]     # beyond the template: plateau
        w = np.linspace(0.5, 1.0, k)              # recent reps weigh more
        err = np.sum(w * (vn_obs[:k] - pred) ** 2)
        if err < best_err:
            best, best_err = N, err
    return best - k


def consistency(sets):
    """Spread between sets when aligned from the start vs from failure."""
    def spread(align):
        rows = []
        for s in sets.values():
            for j, val in enumerate(s["vn"]):
                key = j + 1 if align == "rep" else s["N"] - 1 - j
                rows.append((key, val))
        d = pd.DataFrame(rows, columns=["k", "v"])
        g = d.groupby("k").v.agg(["std", "count"])
        return g[g["count"] >= 5]["std"].mean()
    return spread("rep"), spread("rir")


def main():
    df, sets = load()
    names = list(sets)

    # ---------------- 1. shape consistency -----------------------------------
    sd_rep, sd_rir = consistency(sets)
    print(f"Between-set spread of normalised MCV: aligned by rep {sd_rep:.3f}, "
          f"aligned by RIR {sd_rir:.3f}")
    T_all = template(sets, names)
    print("Average decline-to-failure template (RIR 8..0):",
          np.round(T_all[::-1], 2).tolist())

    # ---------------- 2. early prediction ------------------------------------
    full = df
    rows = []
    for test in names:
        train = [n for n in names if n != test]
        tr = full[full.trial != test]
        N_mean = np.mean([sets[n]["N"] for n in train])
        bB = np.polyfit(tr.vl_pct, tr.rir, 1)
        T = template(sets, train)
        s = sets[test]
        g = full[full.trial == test].sort_values("rep")
        for k in range(4, s["N"] + 1):
            actual = s["N"] - k
            vl_k = g.vl_pct.iloc[k - 1]
            # slope model: fit on training sets at the same k
            xs, ys = [], []
            for n in train:
                if sets[n]["N"] >= k:
                    xs.append(np.polyfit(np.arange(k), sets[n]["vn"][:k], 1)[0])
                    ys.append(sets[n]["N"] - k)
            sl = np.polyfit(np.arange(k), s["vn"][:k], 1)[0]
            pS = np.polyval(np.polyfit(xs, ys, 1), sl) if len(xs) >= 3 else N_mean - k
            rows.append(dict(
                trial=test, side=s["side"], k=k, actual=actual,
                A_count=max(0, N_mean - k),
                B_vel_loss=max(0, np.polyval(bB, vl_k)),
                S_slope=max(0, pS),
                T_template=template_predict(s["vn"], T, k),
            ))
    p = pd.DataFrame(rows)
    p.round(2).to_csv(OUT / "early_rir_predictions.csv", index=False)
    models = ["A_count", "B_vel_loss", "S_slope", "T_template"]

    print("\nMAE (reps) by how many reps have been seen:")
    tab = p.groupby("k")[models].apply(lambda g: (g.sub(p.loc[g.index, "actual"], axis=0)).abs().mean())
    tab["n_sets"] = p.groupby("k").size()
    print(tab.round(2).to_string())

    print("\nAt k = 6 (after six reps):")
    k6 = p[p.k == 6][["trial", "side", "actual"] + models].round(1)
    print(k6.to_string(index=False))

    # ---------------- figures ------------------------------------------------
    fig, axs = plt.subplots(1, 2, figsize=(13, 4.6), sharey=True)
    marker = {1: "o", 2: "s", 3: "^"}
    for n, s in sets.items():
        kw = dict(color=SIDE_C[s["side"]], marker=marker[s["set"]], ms=5, lw=1.5, alpha=0.9,
                  label=f"{'Left' if s['side'] == 'L' else 'Right'} set {s['set']}")
        axs[0].plot(np.arange(1, s["N"] + 1), s["vn"], **kw)
        axs[1].plot(s["N"] - 1 - np.arange(s["N"]), s["vn"], **kw)
    axs[1].plot(range(MAX_RIR_T + 1), T_all, color=INK, lw=3, alpha=0.75, label="average template")
    axs[0].set(title=f"Aligned from the start (spread {sd_rep:.2f})", xlabel="rep",
               ylabel="velocity / reps 1-3 average")
    axs[1].set(title=f"Aligned from failure (spread {sd_rir:.2f})", xlabel="reps in reserve")
    axs[1].invert_xaxis()
    for a in axs:
        a.axhline(1, color=MUTED, lw=0.8)
        a.xaxis.get_major_locator().set_params(integer=True)
    axs[1].legend(fontsize=8, ncol=2, loc="lower left")
    fig.tight_layout()
    fig.savefig(OUT / "decline_shape.png", dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4.4))
    colors = {"A_count": MUTED, "B_vel_loss": C_L, "S_slope": "#1baf7a", "T_template": C_R}
    labels = {"A_count": "A: count prior", "B_vel_loss": "B: velocity loss",
              "S_slope": "S: slope of reps 1..k", "T_template": "T: template match"}
    ks = tab.index[tab.n_sets >= 6]
    for m in models:
        ax.plot(ks, tab.loc[ks, m], marker="o", lw=2, color=colors[m], label=labels[m])
    ax.set(title="Prediction error vs reps observed (leave-one-set-out)",
           xlabel="reps observed so far (k)", ylabel="mean absolute error (reps)")
    ax.set_ylim(bottom=0)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "early_rir_error.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
