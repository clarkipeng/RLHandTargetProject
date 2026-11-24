# A farther objective

The PI asked (2026-11-23) whether pushing the targets farther out would make the policy learn a better throwing motion.
This round fine-tunes the no-squirt policy (`nat_squirt`, variant e of [NATURAL.md](NATURAL.md)) for 8M more steps with the distance ramp allowed to run from 70 out to 92, and measures it on the standard held-out set and on a far one.

[![a far held-out hit](media/far/far-hit.jpg)](media/far/far-hit.mp4)

[far-hit.mp4](media/far/far-hit.mp4): one far-band held-out throw and hit by `far_squirt`, locked camera, 0.4× for 0.1 s around the release and for the 0.12 s before the hit, then the impact held 1.5 s.
The arm is small in the frame because the frame has to hold a target 76 to 92 units out.

## Result: no better, and no new motion

| | held-out, standard set | held-out, far set (76/82/88/92) | carry, far set (mean / max) | backswing (far) | release-speed share s/e/w/f (standard) |
|---|---|---|---|---|---|
| `nat_squirt` (6M) | 62% (72/80/70/19) | 11% (30/13/0/0) | 75.6 / 93.5 | 59° | 0.81 / −0.08 / 0.04 / 0.23 |
| `far_squirt` (6M + 8M) | 58% (69/70/86/2) | 12% (29/2/17/0) | 67.6 / 97.4 | 52° | 0.85 / −0.01 / 0.09 / 0.08 |

240 held-out throws per cell, held start, deterministic actions, both policies run with the same eval on 2026-11-24.
The standard set is the one every earlier doc uses; `ml/eval.py` labels its four bands 30/44/58/70 by the targets' horizontal distance.
The far set moves the four distances to 76/82/88/92 with the same heights and azimuth bands (`--set heldout_d0=76 … heldout_d3=92`).

The far hit rate is unchanged within noise (29 against 27 of 240).
On the standard set it trades the 70-out band (19% to 2%) for the 58-out band (70% to 86%).
The elbow still does not extend at release (share −0.01), so the throw is still a shoulder swing; the fingers' share fell further, from 0.23 to 0.08.

## Why the ramp stalled

`ReachRamp` moves the frontier 1.5 units out each time 64 frontier throws hit at the threshold, here lowered to 30% (`ml/config/far_squirt.yaml`).
The frontier went from 70 to 78 in the first 5.5M steps and to 79.7 by 8M, with frontier throws hitting 15 to 25%, so the policy never trained on 88 to 92.
Half of every lesson's targets are still drawn anywhere from 22 out, so most of the 8M steps went to distances it already hit.

The probe behind the 92 ceiling (comment in the config) found that a direct swing of this arm cannot reach past 72 and that 92 takes a wind-up.
The policy's far throws do wind up (52° backswing) but still swing from the shoulder with the elbow held, and no term in the reward asks for the elbow to extend into the release.
A farther target alone does not supply that gradient: the reward is hit or miss at the target, and a near-miss at 85 pays nothing more than a throw that drops at 60.

## What would

- A distance-graded reward for the far band (pay for carry toward the target, not only the hit), so a longer throw is better than a short one before it can hit.
- A release-time term on elbow extension speed, the joint a human throw adds last.
- A longer run with the near half of the lesson dropped, so the training steps go to the frontier.

Run: `tools/train/train.sh far_squirt --init nat_squirt --gpu --config far_squirt.yaml`, 3.3 h on the shared machine; checkpoint `ml/results/far_squirt/Throw/Throw-8000042.onnx` (not committed).
