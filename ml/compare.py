"""Old arm vs capped arm, old policy vs new: a joint chart for the wall and a markdown table for the docs.

    uv run python compare.py results/windup/compare.jsonl joints.png [--table]

Each row of compare.jsonl is one 240-throw held-out evaluation (eval.py --out ... --tag ...); the last row per tag wins.
The chart has three panels sharing the joint axis: each joint's share of the ball's release speed (telescoping point
velocities, the four shares sum to 1; policies that throw in under half their episodes are left out), the joint's peak speed, and its peak acceleration (over a 1/30 s window).
"""
import json, sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

src, out = Path(sys.argv[1]), Path(sys.argv[2])
rows = {}
for line in open(src):
    r = json.loads(line); rows[r["tag"]] = r
rows = list(rows.values())
JOINTS = ["shoulder", "elbow", "wrist", "fingers"]
CAPS = {"shoulder": 540, "elbow": 540, "wrist": 300, "fingers": 600}  # the capped arm's joint speed limits (deg/s)
# Peak speed in the throw (a window around the release) when the row has it; older rows only have the episode peak.
def peak(r): return r.get("throw_peak", r["peak_speed"])

if "--table" in sys.argv:
    def f(v, p=0): return f"{v:.{p}f}"
    print("| | " + " | ".join(r["tag"] for r in rows) + " |")
    print("|---|" + "---|" * len(rows))
    def line(name, fn): print(f"| {name} | " + " | ".join(fn(r) for r in rows) + " |")
    line("held-out hit rate", lambda r: f"{r['hit_rate']*100:.0f}% ({r['hits']}/{r['episodes']})")
    for b in sorted({b for r in rows for b in r["bands"]}, key=float):
        line(f"hit rate at {b}", lambda r: f"{r['bands'][b]['hit_rate']*100:.0f}%" if b in r["bands"] else "-")
    line("grasp / lift", lambda r: f"{r['grasp_rate']*100:.0f}% / {r['lift_rate']*100:.0f}%")
    line("throw distance mean / max", lambda r: f"{f(r['thrown'],1)} / {f(r['thrown_max'],1)}")
    line("release speed", lambda r: f(r["release_speed"], 1))
    line("backswing (deg)", lambda r: f(r["backswing"]))
    line("wind-up time before release (s)", lambda r: f(r["windup"], 2))
    line("release-speed share s/e/w/f", lambda r: " / ".join(f(r["share"][j], 2) for j in JOINTS))
    line("peak speed s/e/w/f (deg/s)", lambda r: " / ".join(f(peak(r)[j]) for j in JOINTS))
    line("peak speed max s/e/w/f (deg/s)", lambda r: " / ".join(f(r.get("throw_peak_max", r.get("peak_speed_max", {})).get(j, float("nan"))) for j in JOINTS))
    line("peak accel s/e/w/f (k deg/s2)", lambda r: " / ".join(f(r["peak_accel"][j] / 1000) for j in JOINTS))

rows = [r for r in rows if r["throw_rate"] >= 0.5]  # the chart compares policies that throw; a policy that mostly drops the ball is table-only
BG, INK, DIM, GRID = "#0c0e12", "#e9e6e0", "#9aa0a8", "#262a31"
COLORS = ["#3987e5", "#d95926", "#199e70", "#c98500", "#9a5cd0"]  # categorical slots 1-5
plt.rcParams.update({"font.family": "DejaVu Sans", "text.color": INK, "axes.labelcolor": DIM, "xtick.color": DIM,
                     "ytick.color": DIM})
panels = [("share", "share of release speed", 1.0, "{:.2f}"),
          ("peak_speed", "peak joint speed (deg/s)", 1.0, "{:.0f}"),
          ("peak_accel", "peak joint acceleration (k deg/s²)", 1e-3, "{:.0f}")]
fig, axes = plt.subplots(1, 3, figsize=(15, 6), dpi=160, facecolor=BG)
x = np.arange(len(JOINTS)); w = 0.8 / len(rows)
for ax, (key, title, scale, fmt) in zip(axes, panels):
    ax.set_facecolor(BG)
    for s in ax.spines.values(): s.set_visible(False)
    ax.grid(True, axis="y", color=GRID, lw=0.8); ax.set_axisbelow(True)
    for i, r in enumerate(rows):
        v = [(peak(r) if key == "peak_speed" else r[key])[j] * scale for j in JOINTS]
        ax.bar(x + (i - (len(rows) - 1) / 2) * w, v, w * 0.92, color=COLORS[i], label=r["tag"], edgecolor=BG, lw=1)
    ax.axhline(0, color=DIM, lw=1)
    if key == "peak_speed":
        for k, j in enumerate(JOINTS):
            ax.hlines(CAPS[j], k - 0.45, k + 0.45, color=INK, lw=1.6, linestyles=(0, (3, 2)))
        ax.text(len(JOINTS) - 0.5, CAPS["fingers"], " cap", color=INK, fontsize=10, va="center")
    ax.set_xticks(x, JOINTS); ax.set_title(title, loc="left", color=INK, fontsize=13, pad=10)
axes[0].text(0, 1.13, "Where the release speed comes from", transform=axes[0].transAxes, fontsize=17, color=INK)
fig.legend(*axes[0].get_legend_handles_labels(), loc="lower center", ncol=len(rows), frameon=False, fontsize=11)
fig.tight_layout(rect=(0, 0.07, 1, 0.95)); fig.savefig(out, facecolor=BG); print(out)
