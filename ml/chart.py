"""Hit-rate chart for the wall, against environment steps.

    uv run python chart.py results/<run> out.png [evals.jsonl]

The third argument picks the held-out rows to plot (default results/<run>/eval.jsonl, the watcher's 48-throw evals;
results/<run>/stages.jsonl holds 240-throw evals of every checkpoint).

Top: training grasp/lift/throw/hit rates (TensorBoard Task/*) and held-out evaluations, overall and per held-out distance
(25, 33, 42, 52) when the eval rows carry them. Bottom, on the same steps, for runs with the distance ramp: the ramp's
current reach (Task/Reach) and the mean sampled target distance (Task/TargetDistance).
"""
import json, sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

run, out = Path(sys.argv[1]), Path(sys.argv[2])
scalars = {}
for ev in sorted(run.glob("Throw/events.out.tfevents.*")):
    acc = EventAccumulator(str(ev), size_guidance={"scalars": 0}); acc.Reload()
    for tag in acc.Tags()["scalars"]:
        scalars.setdefault(tag, []).extend((e.step, e.value) for e in acc.Scalars(tag))
src = Path(sys.argv[3]) if len(sys.argv) > 3 else run / "eval.jsonl"
evals = [json.loads(l) for l in open(src)] if src.exists() else []
evals = [e for e in evals if e.get("step") is not None and e.get("tag") in (None, "stage") and not e.get("set")]  # skip one-off comparisons
evals = sorted({e["step"]: e for e in evals}.values(), key=lambda e: e["step"])

BG, INK, DIM, GRID = "#0c0e12", "#e9e6e0", "#9aa0a8", "#262a31"
BANDS = {"25": "#3987e5", "33": "#d95926", "42": "#199e70", "52": "#c98500"}  # categorical slots 1-4, validated on BG
plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": INK, "axes.labelcolor": DIM, "xtick.color": DIM, "ytick.color": DIM})

def series(tag, scale=1.0):
    pts = sorted(scalars.get(tag, []))
    return [p[0] / 1e6 for p in pts], [p[1] * scale for p in pts]

ramp = bool(scalars.get("Task/Reach"))
fig, axes = plt.subplots(2 if ramp else 1, 1, figsize=(12, 8.5 if ramp else 6.75), dpi=160, facecolor=BG, sharex=True,
                         gridspec_kw={"height_ratios": [3, 1]} if ramp else None, squeeze=False)
ax = axes[0, 0]
for a in axes[:, 0]:
    a.set_facecolor(BG)
    for s in a.spines.values(): s.set_visible(False)
    a.grid(True, color=GRID, lw=0.8); a.set_axisbelow(True)

for tag, label, color, lw in [("Task/Grasp", "grasp", "#4a515c", 1.4), ("Task/Lift", "lift", "#646c78", 1.4),
                              ("Task/Throw", "throw", "#8a929e", 1.4), ("Task/Hit", "hit (training targets)", INK, 2.0)]:
    x, y = series(tag, 100)
    if x: ax.plot(x, y, color=color, lw=lw, label=label)
if evals:
    ex = [e["step"] / 1e6 for e in evals]
    ax.plot(ex, [e["hit_rate"] * 100 for e in evals], "o-", color=INK, mfc=BG, mew=2, ms=8, lw=1, label="held-out, all distances")
    for band, color in BANDS.items():
        pts = [(e["step"] / 1e6, e["bands"][band]["hit_rate"] * 100) for e in evals if band in e.get("bands", {})]
        if pts: ax.plot(*zip(*pts), "o-", color=color, ms=8, lw=2, mec=BG, mew=1.5, label=f"held-out at {band}")
    best = max(evals, key=lambda e: e["hit_rate"])
    n = evals[-1]["episodes"]
    ax.set_title(f"Held-out hit rate {evals[-1]['hit_rate']*100:.0f}% (best {best['hit_rate']*100:.0f}%, {n} throws per point)",
                 loc="left", fontsize=17, color=INK, pad=14)
else:
    ax.set_title("Throw curriculum", loc="left", fontsize=17, color=INK, pad=14)
lx, ly = series("Task/Lesson")
if lx:
    names = ["grasp", "lift", "throw", "reach"] if ramp else ["grasp", "lift", "throw", "near", "mid", "far"]
    prev, last_x = None, -1e9
    for x, y in zip(lx, ly):
        k = min(int(round(y)), len(names) - 1)
        if k != prev:
            for a in axes[:, 0]: a.axvline(x, color=GRID, lw=1.2, ls="--")
            if x - last_x > 0.04 * max(lx):  # skip labels that would overprint the previous one
                ax.text(x, 101, f" {names[k]}", color=DIM, fontsize=10, va="bottom"); last_x = x
            prev = k
ax.set_ylim(0, 108); ax.set_ylabel("% of episodes")
ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), frameon=False)
if ramp:
    rx = axes[1, 0]
    x, y = series("Task/Reach"); rx.plot(x, y, color=INK, lw=2, label="reach (farthest target)")
    x, y = series("Task/TargetDistance"); rx.plot(x, y, color=DIM, lw=1.2, ls=":", label="mean target distance")
    rx.set_ylim(15, 60); rx.set_ylabel("target distance")
    rs = sorted(scalars["Task/Reach"])
    for e in evals:  # the ramp's reach at each evaluated checkpoint
        r = min(rs, key=lambda p: abs(p[0] - e["step"]))[1]
        rx.plot(e["step"] / 1e6, r, "o", color=INK, mfc=BG, mew=1.5, ms=6)
        rx.annotate(f"{r:.0f}", (e["step"] / 1e6, r), textcoords="offset points", xytext=(0, 7), ha="center", color=DIM, fontsize=9)
    rx.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), frameon=False)
axes[-1, 0].set_xlabel("environment steps (millions)")
fig.tight_layout(); fig.savefig(out, facecolor=BG); print(out)
