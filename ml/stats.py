"""Latest training scalars of a run: uv run python stats.py results/<run> [tags...]"""
import glob, sys
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
run = sys.argv[1]
tags = sys.argv[2:] or ["Environment/Cumulative Reward", "Environment/Episode Length", "Environment/Lesson Number/lesson",
                        "Task/Grasp", "Task/Lift", "Task/Throw", "Task/Hit", "Task/ThrowDistance", "Policy/Entropy"]
series = {}
for f in sorted(glob.glob(f"{run}/Throw/events.out.tfevents.*")):
    acc = EventAccumulator(f, size_guidance={"scalars": 0}); acc.Reload()
    for t in acc.Tags()["scalars"]:
        series.setdefault(t, []).extend(acc.Scalars(t))
last = max((e.step for v in series.values() for e in v), default=0)
print(f"step {last/1e6:.1f}M")
for t in tags:
    v = sorted(series.get(t, []), key=lambda e: e.step)
    if v: print(f"  {t:36s} " + "  ".join(f"{e.value:7.3f}" for e in v[-6:]))
