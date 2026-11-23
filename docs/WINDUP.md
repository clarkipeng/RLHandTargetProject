# Teaching the arm to wind up

## Round 2: the wind-up emerges

The policy now winds up through the shoulder before every throw: a 40° backswing (p90 63°) over 1.1 s, then a forward swing of 65°.
It hits 63% of 240 held-out throws at 30/44/58/70 out (75/83/66/25%), throws 48 on average and up to 84, on an arm whose joint speed caps now hold in the throw.
**No shaping reward produced it.**
The variant that worked is the start, not a bonus: the ball held in a forward-pointing hand, targets ramped out past the farthest no-backswing throw (60), and a ramp that keeps moving.
The annealed backswing bonus (task step 4, `windup_bonus` in `throw_windup_bonus.yaml`) was built but the final run, `windup13`, trained from scratch with it off.

[![the wind-up at 70 out](media/windup2/far.jpg)](media/windup2/far.mp4)

[far.mp4](media/windup2/far.mp4): `windup13` at 40M on the 70-out held-out target (azimuth 30), 0.25× from 1.2 s before the release, locked camera.

### 1. Making the caps real

PR #8's caps limited drive force through a torque-speed line, and joints still ran far past them.
Three causes, in order of size:

- **The position drive.** A stiff position drive asks for any velocity it likes; the force limit only slows how fast it gets there. The drives are now velocity servos (`drive_mode 1`): the action sets a target speed clamped to the cap, and the torque cap bounds the force that reaches it.
- **Contacts and passive whip.** Finger contacts with the ball, and the wrist whipped by the swinging forearm, add speed that no drive asked for. `maxJointVelocity` plus a clamp of each joint's velocity after every physics step (`speed_enforce 1`) bounds that. A governor that also cut the drive near the cap (`speed_enforce 2`) fought the finger contacts and blew the wrist and fingers up to 1600–2700 deg/s (27% hits); it is off.
- **One step of gain.** PhysX integrates a joint with its end-of-step velocity, so a joint clamped to exactly its cap still gains in the next step. The predictive clamp (`speed_predict 1`) leaves room for the gain the last step added in the same direction. On the 38M checkpoint it took the wrist's worst throw from 388 to 343 deg/s (96 throws) at the same hit rate.

Measured in the throw (from 0.5 s before to just after the release), worst case of 240 throws, against the cap: shoulder 417 / 540, elbow 186 / 540, wrist 360 / 300 (p90 330), fingers 642 / 600 (p90 563).
PR #8's fingers peaked at 2074.
The wrist still runs up to 20% over in the last steps before release, and the fingers 7%.
The `peak_sustained` finger figure of ~1960 in `eval.jsonl` is not the throw: it is the hold setup at the episode start, when the fingers close on the ball before it is released to physics, several seconds before the throw.

### 2. Proving a wind-up is needed

`ml/probe.py` searches open-loop action plans with CMA-ES on the capped arm from the start pose and records how far the ball would carry (drag-free) at its best release.
Two searches, the same arm and pose:

| plan | best carry |
|---|---|
| direct: backswing under 5° | 60.1 |
| free: any backswing (it found 62°) | 92.8 |

A direct swing cannot reach past about 60, so the ramp's far end is 72 (`reach_max`) and the held-out bands are 30/44/58/70 (PR #8: 25/33/42/52).
The 58 band is at the edge of a direct throw, and 70 is clearly beyond it.

### 3. Giving it room

- **Held start** (`start_held 1`): the ball begins closed in the hand, the pedestal lowered out of the swing (`ped_y -14.5`). PR #8's ball started low on a pedestal, so the lift already was the acceleration path and a backswing meant lowering it again.
- **Forward start pose** (shoulder at -0.5): the arm starts pointing forward, so swinging back is the natural first motion.
- **Counterbalanced arm** (`arm_gravity 0`): the capped shoulder no longer spends its torque holding the arm up.
- **10 s episodes**, of which the wind-up and throw take about 1.5 s.
- **Finger actions relative to the closed grip** (`pose_relative 1`): a zero action keeps the ball held, so exploring the arm does not drop it.

### 4. A ramp that keeps moving

The ramp moves the targets out by 1.5 each time 64 throws at the frontier hit `reach_threshold` of the time.
At 60% it stalled near reach 60: about a third of training throws are stochastic early releases, while the deterministic policy hit far more.
At 45% (`reach_threshold 0.45`), starting from 18 (`reach_start`), it climbed +3 per 1M steps.
The backswing grew with the reach, from 2° at reach 20 to 29° at reach 65 (training, stochastic).
`windup13` was stopped at 40M with the reach at 66.8, climbing +0.5 per 1M steps.

### Results

![joint shares](media/windup2/joints.png)

Measured as in PR #8, 240 held-out throws, deterministic:

| | PR #8 (no wind-up) | round 2 (`windup13` 40M) |
|---|---|---|
| held-out bands | 25 / 33 / 42 / 52 | 30 / 44 / 58 / 70 |
| hit rate per band | 89 / 87 / 90 / 78% | 75 / 83 / 66 / 25% |
| hit rate | 86% | 63% |
| throw distance mean / max | 35.7 / 71.0 | 47.8 / 83.9 |
| backswing (p90) | 13° | 40° (63°) |
| wind-up time before release | 0.31 s | 1.07 s |
| release-speed share s/e/w/f | -0.37 / 0.66 / 0.03 / 0.67 | 0.48 / -0.08 / 0.00 / 0.59 |
| peak speed in the throw s/e/w/f (deg/s) | 410 / 210 / 344 / 2074 (episode) | 300 / 142 / 309 / 533 |
| worst throw s/e/w/f (deg/s) | - | 417 / 186 / 360 / 642 |

PR #8's policy cannot be run on this arm (servo drives, counterbalance, held start): it grasps but never lifts, so the PR #8 column is its own 240-throw evaluation on its own bands.

Checkpoints of `windup13` (96 throws each): 19M 34% (backswing 31°), 28M 57% (38°), 38M 58% (37°), 40M 63% at 240 throws.

What is weak:

- **The shoulder winds up, the elbow doesn't.** The shoulder went from -0.37 to 0.48 of the release speed, but the elbow's share is slightly negative, and the fingers still give 0.59.
- **The 70 band is mostly misses on aim, not distance.** Throws there average 72, but the hits are almost all at azimuths 30° and 50° (79% and 46% in 24-throw checks); at -50° to 10° they overshoot or fall short.
- **The wrist leaks up to 20% over its cap** just before release.
- **Hit rate is lower than PR #8's** (63% against 86%), on bands 5 to 18 farther out; even the nearest band (75% at 30) is below PR #8's 78% at 52.

## Round 1 (PR #8): a weaker arm and a distance ramp

The PR #6 arm could hit a target 55 out with a short wrist and finger flick.
This change caps every joint's torque and speed, the wrist and fingers hardest, and replaces the near/mid/far lessons with a distance ramp that moves the targets out in 1.5-unit steps as the policy earns them.
A policy retrained on the capped arm hits 86% of the 240 held-out throws (PR #6: 91% on its own arm), and 78% at 52 out.
On the capped arm the PR #6 policy hits 0%.

**The wind-up the task asked for did not emerge.**
The new policy takes the distance from the elbow (two thirds of the release speed, up from 45%) and the wrist no longer contributes, but its backswing is smaller than PR #6's (13° against 19°).
Read the "Negative result" section below before building on this.

![hit rate by stage](media/windup/stages.png)

### The arm limits

`ArmLimits` in `Assets/Training/Runtime/ThrowAgent.cs` sets each joint group's drive force limit every physics step from a torque-speed line.
The full torque is available at rest and falls linearly to zero at the speed cap.
Above the cap the drive brakes.

| group | torque | speed cap (deg/s) |
|---|---|---|
| shoulder | 2000 | 540 |
| elbow | 1000 | 540 |
| wrist | 400 | 300 |
| fingers | 150 | 600 |

The `arm_limits` environment parameter switches the caps off (0, the PR #6 arm).
`torque_<group>` and `speed_<group>` override single values for sweeps.

### Two rounds

**Round 1** (run `windup02`) capped the shoulder at 4000 and the elbow at 2000.
Its policy hit 70% held-out on that arm with throws up to 73 out, so a direct swing still reached the far band and nothing pushed it toward a wind-up.

Diagnostics on the round-1 policy (96 held-out throws each; 66% on the unchanged round-1 arm):

- Halving the finger torque cut the hit rate from 66% to 32%, but the farthest throw stayed at 75. The fingers aim the throw; they don't supply the distance.
- Capping the finger speed hard (5 deg/s) broke the lift (8%).

A shoulder/elbow sweep found the strength where a direct swing falls short:

| shoulder / elbow torque | round-1 policy hit rate (96 throws) | notes |
|---|---|---|
| 4000 / 2000 | 66% | |
| 3000 / 1500 | 57% | |
| 2500 / 1200 | 29% | |
| 2000 / 1000 | 14% | lift 64%; mean throw on the 52 targets is 47 (short) |

**Round 2** (run `windup03`) uses 2000 / 1000.
It was initialised from `windup02` and trained 12M steps on the GPU (`train.sh --gpu`, 12 players × 8 arenas, about 6M steps per hour).

### The distance ramp

`ReachRamp` in `ThrowAgent.cs` replaces the near/mid/far lessons with one `reach` lesson. It works as follows:

- The reach starts at 26.
- After every 64 frontier episodes, the reach grows by 1.5 if at least 60% of them hit, up to 56.
- Half of the targets sit in the frontier band (reach − 4 to reach). The other half fall anywhere from 22 to the reach, so near throws keep being practised.
- The target radius narrows from 6 to the final 4 by reach 40.

The ramp lives in each player process.
`train.sh --resume` reads the last `Task/Reach` from TensorBoard and starts the players there (`-reachStart`).
TensorBoard logs `Task/Reach` and `Task/TargetDistance`, and the lower panel of the chart above plots both.

In `windup03` the reach was 26 at 1M steps, 32 at 2M, 38 at 3M, 45 at 4M, 50 at 5M, 54 at 6M, and 56 by 6.8M.

| steps | held-out hit rate (240 throws) | at 25 | at 33 | at 42 | at 52 |
|---|---|---|---|---|---|
| 1M | 17% | 44% | 20% | 0% | 0% |
| 3M | 62% | 85% | 79% | 73% | 10% |
| 7M | 78% | 84% | 86% | 73% | 67% |
| 10M | 86% | 89% | 87% | 90% | 78% |
| 12M | 82% | | | | |

The 10M checkpoint (`windup03/Throw/Throw-9999898.onnx`) is the result.
All stage rows are in `ml/results/windup03/stages.jsonl` (copied to `docs/media/windup/stages.jsonl`; the comparison rows to `docs/media/windup/compare.jsonl`).

### Old vs new

Each column is 240 held-out throws (`docs/media/windup/compare.jsonl`).
Per-throw measures average the episodes where the ball was thrown.

| | PR6 policy, PR6 arm | PR6 policy, capped arm | new policy, capped arm |
|---|---|---|---|
| held-out hit rate | 91% (218/240) | 0% (0/240) | 86% (206/240) |
| hit rate at 25 | 94% | 0% | 89% |
| hit rate at 33 | 89% | 0% | 87% |
| hit rate at 42 | 97% | 0% | 90% |
| hit rate at 52 | 84% | 0% | 78% |
| grasp / lift | 100% / 100% | 58% / 12% | 100% / 100% |
| throw distance mean / max | 34.3 / 55.8 | 21.3 / 30.3 | 35.7 / 71.0 |
| release speed | 23.8 | 13.2 | 19.4 |
| backswing (deg) | 19 | 19 | 13 |
| wind-up time before release (s) | 0.25 | 0.59 | 0.31 |
| release-speed share s/e/w/f | -0.38 / 0.45 / -0.30 / 1.23 | -0.72 / 0.83 / 0.31 / 0.58 | -0.37 / 0.66 / 0.03 / 0.67 |
| peak speed s/e/w/f (deg/s) | 459 / 257 / 506 / 1300 | 386 / 174 / 431 / 1070 | 410 / 210 / 344 / 2074 |
| peak accel s/e/w/f (k deg/s2) | 11 / 7 / 15 / 39 | 8 / 4 / 15 / 32 | 8 / 3 / 12 / 64 |

How the measures are defined:

- **Shares:** the ball's release velocity is split along the chain (shoulder, elbow, wrist, fingers) by telescoping point velocities. The four shares sum to 1; a negative share means that joint moves the hand against the throw.
- **Backswing:** the largest angle the hand swings back, away from the target, before release.
- **Wind-up time:** the time from the deepest point of the backswing to release.
- **Peak acceleration:** measured over a 1/30 s window.

On the capped arm the PR #6 policy mostly fails to lift the ball (12%). It never hits, and its farthest throw is 30.

![joint shares](media/windup/joints.png)

### Negative result: no wind-up

What changed between PR #6 and the new policy:

- The wrist's share fell from −0.30 to 0.03.
- The fingers' share fell from 1.23 to 0.67.
- The elbow now supplies two thirds of the release speed.

What didn't change:

- The backswing stayed at 15–16° through all of `windup03`'s checkpoints and ends at 13°, smaller than PR #6's.
- Release speed dropped (19.4 against 23.8), yet throws still reach 71. The policy throws higher rather than harder.

Three weaknesses to know about:

- **The finger speed cap doesn't hold.** Finger peaks reach 2000 deg/s against a 600 cap. The cap limits drive force, not contact dynamics, and the fingers are still flicking the ball.
- **The ball starts low,** so the upswing from the pedestal already gives a long acceleration path. A backswing would mean first lowering a ball that is already low.
- **Weakening further breaks the task.** Below 2000 / 1000 the arm can no longer lift the ball reliably. The next lever would be a different start (ball held at shoulder height) or an explicit reward on backswing, not more torque cuts.

No acceleration or jerk penalty was added: with the torque-speed caps, the shoulder and elbow peaks already fell, and a penalty would push further away from a wind-up.

### Clips

- [sbs.mp4](media/windup/sbs.mp4): PR #6 on its arm against the new policy on the capped arm, at the 52-out held-out target, slowed to 0.2× through the release.
- [watch_it_learn.mp4](media/windup/watch_it_learn.mp4): `windup03` checkpoints at 1M, 3M, 5M, 7M and 10M, on the 52-out held-out target. Each is labelled with its training reach and its 240-throw hit rate at 52, with a live hits/attempts counter.

## Commands

```
tools/train/train.sh windup03 --init windup02 --gpu        # capped arm, distance ramp
tools/train/train.sh windup03 --resume --gpu               # resumes at the last logged reach
uv run --project ml python ml/eval.py <policy.onnx> --episodes 240 [--limits old] [--set torque_elbow=1500] --out compare.jsonl --tag "..."
uv run --project ml python ml/compare.py ml/results/windup/compare.jsonl joints.png [--table]
uv run --project ml python ml/chart.py ml/results/windup03 stages.png ml/results/windup03/stages.jsonl
tools/train/sidebyside.sh out.mp4 "PR6 policy, PR6 arm" ml/results/throw09/Throw/Throw-15999874.onnx old \
    "new policy, capped arm" ml/results/windup03/Throw/Throw-9999898.onnx new --target 20
LEARN_TARGET=20 LEARN_SECONDS=12 tools/train/learn.sh out.mp4 "1M, reach 26=ml/results/windup03/Throw/Throw-999888.onnx" ...
```

Round 2:

```
tools/train/train.sh windup13 --gpu --config throw_held_ramp.yaml      # held start, forward pose, ramp 18 -> 72
tools/train/train.sh windup13 --resume --gpu --config throw_held_ramp.yaml
uv run --project ml python ml/probe.py ...                              # direct vs wind-up carry (results/windup2/probe.jsonl)
uv run --project ml python ml/eval.py ml/results/windup13/Throw/Throw-39999954.onnx --episodes 240 --arenas 8 \
    --set start_held=1 --set ped_y=-14.5 --set pose_relative=1 --set speed_predict=1
uv run --project ml python ml/compare.py docs/media/windup2/compare.jsonl joints.png --table
tools/win/unity.sh --graphics -executeMethod Showcase.Editor.ReplayRenderer.RenderFromCommandLine -take far.take -look minimal \
    -reel Assets/Showcase/Reels/windup2.json -out far.mp4 -stills far-stills    # take: eval.py ... --take far.take --set eval_target=22
```
