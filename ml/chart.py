"""Hit-rate chart for the wall: training hit rate (TensorBoard Task/Hit) and held-out evaluations, against steps.

    uv run python chart.py results/<run> out.png
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
evals = [json.loads(l) for l in open(run / "eval.jsonl")] if (run / "eval.jsonl").exists() else []

BG, INK, DIM, GRID = "#0c0e12", "#e9e6e0", "#9aa0a8", "#262a31"
plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": INK, "axes.labelcolor": DIM, "xtick.color": DIM, "ytick.color": DIM})
fig, ax = plt.subplots(figsize=(12, 6.75), dpi=160, facecolor=BG)
ax.set_facecolor(BG)
for s in ax.spines.values(): s.set_visible(False)
ax.grid(True, color=GRID, lw=0.8); ax.set_axisbelow(True)

def series(tag):
    pts = sorted(scalars.get(tag, []))
    return [p[0] / 1e6 for p in pts], [p[1] * 100 for p in pts]

lines = [("Task/Grasp", "grasp", "#5b6573"), ("Task/Lift", "lift", "#7d8796"), ("Task/Throw", "throw", "#a9b2bf"),
         ("Task/Hit", "hit (training targets)", "#ffc468")]
for tag, label, color in lines:
    x, y = series(tag)
    if x: ax.plot(x, y, color=color, lw=2.6 if "hit" in label else 1.6, label=label)
if evals:
    ex = [e["step"] / 1e6 for e in evals if e["step"] is not None]
    ey = [e["hit_rate"] * 100 for e in evals if e["step"] is not None]
    ax.plot(ex, ey, "o", color="#1fc3dc", ms=7, label="hit (held-out targets, eval)")
    best = max(evals, key=lambda e: e["hit_rate"])
    ax.set_title(f"Held-out hit rate {evals[-1]['hit_rate']*100:.0f}% (best {best['hit_rate']*100:.0f}%)",
                 loc="left", fontsize=17, color=INK, pad=14)
else:
    ax.set_title("Throw curriculum", loc="left", fontsize=17, color=INK, pad=14)
lx, ly = series("Task/Lesson")
if lx:
    names = ["grasp", "lift", "throw", "near", "mid", "far"]
    prev = None
    for x, y in zip(lx, ly):
        k = int(round(y / 100))
        if k != prev:
            ax.axvline(x, color=GRID, lw=1.2, ls="--")
            ax.text(x, 101, f" {names[min(k, 5)]}", color=DIM, fontsize=10, va="bottom")
            prev = k
ax.set_ylim(0, 108); ax.set_xlabel("environment steps (millions)"); ax.set_ylabel("% of episodes")
leg = ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5), frameon=False)
fig.tight_layout(); fig.savefig(out, facecolor=BG); print(out)
