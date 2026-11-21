# Training the throw

One PPO policy learns, end to end, to reach a ball resting on a pedestal, grasp it, lift it, wind up, release and hit a target.
It trains headless on the Windows PC: Unity 2022.3.62f3 training players in batchmode, ML-Agents 0.30 `mlagents-learn` in WSL, 12 players with 8 arenas each, on the CPU (the GPU measured slower, since the simulation is the bottleneck).

## Result

The held-out hit rate is 90.8% (218 of 240 throws) at 36M steps.
Held-out targets sit in six azimuth bands (±50°, ±30°, ±10°, each ±3° wide) that the final lessons never sample, at four distance and height pairs, 24 targets in all.

![held-out hit rate](media/train/hit_rate.png)

| overall steps | held-out hit rate | throws |
|---|---|---|
| 14M | 6% | 48 |
| 16M | 17% | 48 |
| 19M | 40% | 48 |
| 20M | 56% | 48 |
| 22M | 67% | 48 |
| 24M | 83% | 48 |
| 28M | 88% | 48 |
| 30M | 89% | 240 |
| 34M | 88% | 240 |
| 36M | 91% | 240 |

From 30M on, every checkpoint scores between 82% and 91% on 240 throws, so the run was stopped at 36M.

## Clips

- [watch_it_learn.mp4](media/train/watch_it_learn.mp4): flailing at 1M, grasping at 7M, lifting at 13M, first throws at 16M and aimed throws at 20M, each labelled with its held-out hit rate and a live hits/attempts counter.
- [eval_01M.mp4](media/train/eval_01M.mp4) and [eval_30M.mp4](media/train/eval_30M.mp4): held-out eval rollouts early and late.
- Cinematic 4K 60 fps reels of the 36M policy: [art](media/renders/throw09-36M/art/reel.mp4) and [minimal](media/renders/throw09-36M/minimal/reel.mp4) (1080p copies here, the 4K masters stay in `renders/`).

![art](media/renders/throw09-36M/art/sheet.jpg)
![minimal](media/renders/throw09-36M/minimal/sheet.jpg)

## Curriculum

The `lesson` environment parameter in `ml/config/throw.yaml` advances on the mean reward.

1. grasp: hold the ball off the pedestal for 0.3 s.
2. lift: hold it 3 units above its rest for 0.3 s.
3. throw: release it into a wide zone.
4. near: targets 22 to 32 out, radius 6.
5. mid: targets 22 to 42 out, radius 5.
6. far: targets 22 to 55 out at varied heights, radius 4 (the final task).

The reward is +1 for each lesson's own success, a bounded shaping potential (reach 0.2, grasp 0.1, lift 0.1), a ballistic release reward from the drag-free predicted miss distance, a landing reward from the closest approach, and +0.5 on a hit.
Holding the ball to the end of a throw episode forfeits the shaping, which is what made the arm let go.

## Runs

The final policy is the result of three chained runs, each initialised from the previous one's checkpoint.
`throw07` trained 14M steps, `throw08` 6M more (stopped by a WSL restart), and `throw09` 16M more.
A run initialised with `--init` replays the curriculum from grasp, which takes it a few minutes to climb back to the far lesson.
`ml/results/<run>/offset` records the steps trained before a run, so wall clips are named by overall steps.

## Commands

```
tools/train/build.sh                          # build the Windows training player
tools/train/train.sh throw09 --init throw08r  # train in the background (12 players x 8 arenas, CPU)
tools/train/watch.sh throw09                  # every 20 min: held-out eval, chart and milestone clips on the wall
uv run --project ml python ml/eval.py ml/results/throw09/Throw/Throw-15999874.onnx --episodes 240
tools/train/learn.sh out.mp4 "label=policy.onnx[@lesson]" ...
tools/showcase/render.sh ml/results/throw09/Throw/Throw-15999874.onnx renders/throw09-36M
```

Peak memory while training was about 2.4 GB (1 GB in WSL, 1.4 GB for the Windows players); renders ran only while training was stopped.
