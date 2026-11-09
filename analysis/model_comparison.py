#!/usr/bin/env python3
"""Does inter-rep rest explain velocity-based RIR errors? Simple vs complex models.

Same scenario as stop_early_rir.py: every to-failure set is truncated at each
rep k >= 3, the model sees reps 1..k only, leave-one-set-out validation.

Feature groups (all computable live at the stopping rep):
  velocity   mcv2 (2-rep avg MCV), vl (velocity loss vs fastest so far)
  rest       rest_k (bottom pause before rep k, s), rest2 (avg of last 2 pauses)
  effort     tcon (concentric time), stall (time stalled), pcv (peak velocity)
  set-clock  elapsed (s since rep 1), k (reps done), fatigue state F_k

The set-clock group grows with every rep, so a model using it can learn "sets
at this load last ~10 reps" - the count prior in disguise. That works on this
data but won't transfer to a different load, so those models are reported
separately.
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.ensemble import GradientBoostingRegressor, RandomForestRegressor
from sklearn.linear_model import Ridge
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

OUT = Path(__file__).resolve().parent / "output"
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e6e5e0"
C1, C2, C3 = "#2a78d6", "#eb6834", "#1baf7a"
plt.rcParams.update({
    "axes.edgecolor": GRID, "axes.labelcolor": MUTED, "xtick.color": MUTED,
    "ytick.color": MUTED, "text.color": INK, "axes.spines.top": False,
    "axes.spines.right": False, "axes.grid": True, "grid.color": GRID,
    "axes.titlelocation": "left", "axes.titlesize": 11, "legend.frameon": False,
})


def fatigue_state(rests, tau):
    """One unit of fatigue per rep, recovering exponentially during pauses."""
    F, out = 0.0, []
    for r in rests:
        F = F * np.exp(-r / tau) + 1.0
        out.append(F)
    return np.array(out)


def build():
    df = pd.read_csv(OUT / "reps.csv")
    f = df[df.full].copy().sort_values(["trial", "rep"])
    rows = []
    for name, g in f.groupby("trial", sort=False):
        v = g.mcv_ang_dps.to_numpy()
        v2 = pd.Series(v).rolling(2, min_periods=1).mean().to_numpy()
        rest = (g.t_start - g.t_end.shift()).fillna(0).clip(lower=0).to_numpy()
        rest2 = pd.Series(rest).rolling(2, min_periods=1).mean().to_numpy()
        rm = np.maximum.accumulate(v)
        for j in range(len(g)):
            rows.append(dict(
                trial=name, side=g.side.iloc[0], k=j + 1, rir=len(g) - 1 - j,
                mcv2=v2[j], vl=100 * (1 - v[j] / rm[j]), rest_k=rest[j], rest2=rest2[j],
                tcon=g.t_con_s.iloc[j], stall=g.stall_s.iloc[j], pcv=g.pcv_ang_dps.iloc[j],
                elapsed=g.t_start.iloc[j] - g.t_start.iloc[0], _rests=rest[: j + 1].tolist()))
    d = pd.DataFrame(rows)
    return d[d.k >= 3].reset_index(drop=True)


def add_F(d, tau):
    d = d.copy()
    d["F"] = [fatigue_state(r, tau)[-1] for r in d._rests]
    return d


# --------------------------------------------------------------------------- #
def linear(feats):
    def fit_pred(tr, te):
        X = np.c_[np.ones(len(tr)), tr[feats]]
        b = np.linalg.lstsq(X, tr.rir, rcond=None)[0]
        return np.c_[np.ones(len(te)), te[feats]] @ b
    return fit_pred


def ridge(feats):
    def fit_pred(tr, te):
        best, best_e = 1.0, np.inf
        for a in (0.3, 1, 3, 10, 30, 100):                # inner leave-one-set-out
            e = []
            for t in tr.trial.unique():
                m = make_pipeline(StandardScaler(), Ridge(alpha=a))
                m.fit(tr[tr.trial != t][feats], tr[tr.trial != t].rir)
                e.extend(np.abs(m.predict(tr[tr.trial == t][feats]) - tr[tr.trial == t].rir))
            if np.mean(e) < best_e:
                best, best_e = a, np.mean(e)
        m = make_pipeline(StandardScaler(), Ridge(alpha=best)).fit(tr[feats], tr.rir)
        return m.predict(te[feats])
    return fit_pred


def forest(feats):
    def fit_pred(tr, te):
        m = RandomForestRegressor(n_estimators=400, min_samples_leaf=3, random_state=0)
        return m.fit(tr[feats], tr.rir).predict(te[feats])
    return fit_pred


def boosting(feats):
    def fit_pred(tr, te):
        m = GradientBoostingRegressor(n_estimators=150, max_depth=2, learning_rate=0.05,
                                      subsample=0.8, random_state=0)
        return m.fit(tr[feats], tr.rir).predict(te[feats])
    return fit_pred


def fatigue_model(extra):
    """Linear on velocity + F_k, with tau chosen inside the training folds."""
    def fit_pred(tr, te):
        best, best_e = None, np.inf
        for tau in (2, 4, 8, 16, 32, 64):
            trF = add_F(tr, tau)
            e = []
            for t in trF.trial.unique():
                p = linear(["mcv2", "vl", "F"] + extra)(trF[trF.trial != t], trF[trF.trial == t])
                e.extend(np.abs(p - trF[trF.trial == t].rir))
            if np.mean(e) < best_e:
                best, best_e = tau, np.mean(e)
        return linear(["mcv2", "vl", "F"] + extra)(add_F(tr, best), add_F(te, best))
    return fit_pred


def loso(d, fp):
    pred = np.empty(len(d))
    for t in d.trial.unique():
        m = (d.trial == t).to_numpy()
        pred[m] = fp(d[~m], d[m])
    return np.clip(pred, 0, None)


# --------------------------------------------------------------------------- #
def main():
    d = build()
    V = ["mcv2", "vl"]
    R = ["rest_k", "rest2"]
    E = ["tcon", "stall", "pcv"]
    C = ["elapsed", "k"]
    models = [
        # name, group, fit_pred, n_params (rough complexity)
        ("current: velocity (linear)", "transferable", linear(V), 3),
        ("velocity + rest before rep (linear)", "transferable", linear(V + ["rest_k"]), 4),
        ("velocity + avg rest last 2 (linear)", "transferable", linear(V + ["rest2"]), 4),
        ("velocity + rest + effort (ridge)", "transferable", ridge(V + R + E), 8),
        ("velocity + rest + effort (random forest)", "transferable", forest(V + R + E), 400),
        ("velocity + rest + effort (boosting)", "transferable", boosting(V + R + E), 150),
        ("velocity + fatigue-recovery state F", "set-clock", fatigue_model([]), 5),
        ("velocity + elapsed time + k (linear)", "set-clock", linear(V + C), 5),
        ("everything (random forest)", "set-clock", forest(V + R + E + C), 400),
    ]
    base = None
    rows, preds = [], {}
    for name, grp, fp, npar in models:
        p = loso(d, fp)
        preds[name] = p
        e = np.abs(p - d.rir)
        per_set = pd.Series(e.to_numpy()).groupby(d.trial).mean()
        if base is None:
            base = per_set
        rows.append(dict(model=name, group=grp, params=npar, MAE=e.mean(),
                         MAE_rir_0_2=e[d.rir <= 2].mean(), within1=100 * (e <= 1).mean(),
                         sets_better=int((per_set < base - 1e-9).sum()),
                         sets_worse=int((per_set > base + 1e-9).sum()),
                         worst_set_MAE=per_set.max()))
    res = pd.DataFrame(rows)
    pd.set_option("display.width", 200)
    print(res.round(2).to_string(index=False))
    res.round(3).to_csv(OUT / "model_comparison.csv", index=False)

    # Residual check: are the current model's errors explained by rest?
    r0 = preds["current: velocity (linear)"] - d.rir
    print(f"\ncorr(current-model error, rest before rep) = {np.corrcoef(r0, d.rest_k)[0, 1]:+.2f}")
    print(f"corr(current-model error, avg rest last 2) = {np.corrcoef(r0, d.rest2)[0, 1]:+.2f}")
    big = d.assign(err=r0).loc[np.abs(r0) >= 1.5, ["trial", "k", "rir", "mcv2", "vl", "rest_k", "err"]]
    print("\nCurrent model's big misses (|error| >= 1.5 reps):")
    print(big.round(2).to_string(index=False))

    # in-sample vs out-of-sample (overfitting gap)
    print("\nIn-sample vs leave-one-set-out MAE:")
    for name, grp, fp, _ in models:
        ins = np.abs(np.clip(fp(d, d), 0, None) - d.rir).mean()
        print(f"  {name:45s} in-sample {ins:.2f}   held-out {res.set_index('model').MAE[name]:.2f}")

    # figure
    fig, ax = plt.subplots(figsize=(10, 5.2))
    col = {"transferable": C1, "set-clock": C2}
    res_ = res.iloc[::-1]
    ax.barh(res_.model, res_.MAE, color=[col[g] for g in res_.group], height=0.6)
    for y, (m, v) in enumerate(zip(res_.model, res_.MAE)):
        ax.text(v + 0.02, y, f"{v:.2f}", va="center", fontsize=9, color=INK)
    ax.axvline(res.MAE.iloc[0], color=MUTED, ls="--", lw=1)
    ax.set(title="RIR error on stopped-early sets (leave-one-set-out)",
           xlabel="mean absolute error (reps)")
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color=C1, label="transfers to other loads"),
                       Patch(color=C2, label="uses set clock (learns this load's set length)")],
              loc="upper center", bbox_to_anchor=(0.35, -0.13), ncol=2, fontsize=9)
    ax.grid(axis="y", visible=False)
    fig.tight_layout()
    fig.savefig(OUT / "model_comparison.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
