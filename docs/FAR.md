# A farther objective

The PI asked (2026-11-23) whether pushing the targets farther out would make the policy learn a better throwing motion.
This round fine-tunes the no-squirt policy (`nat_squirt`, variant e of [NATURAL.md](NATURAL.md)) for 8M more steps with the distance ramp allowed to run from 70 out to 92, and measures it on the standard held-out set and on a far one.
Farther targets alone changed neither the hit rate nor the motion; adding a release-time term for the elbow ([round 2](#round-2-pay-for-the-elbow)) changed both: the elbow now extends into the throw, and held-out hits at 70 out double.

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
Half of every lesson's targets are still drawn anywhere from 22 out, so at least half of the 8M steps went to distances it already hit.

The probe behind the 92 ceiling (comment in the config) found that a direct swing of this arm cannot reach past 72 and that 92 takes a wind-up.
The policy's far throws do wind up (52° backswing) but still swing from the shoulder with the elbow held, and no term in the reward asks for the elbow to extend into the release.
A farther target alone does not supply that gradient.
The reward already pays for a near miss (0.5·exp(−gap/8) at release from the predicted flight, 0.25·exp(−gap/8) at the end from the closest approach), so a throw that falls 7 short of a 92-out target earns about 0.3 more than one that drops 30 short.
The gradient toward farther throws is there; the policy did not find a motion that follows it past about 80 (far throws carry 68 on average, 97 at most).

Run: `tools/train/train.sh far_squirt --init nat_squirt --gpu --config far_squirt.yaml`, 3.3 h on the shared machine; checkpoint `ml/results/far_squirt/Throw/Throw-8000042.onnx` (not committed).

## Round 2: pay for the elbow

`elbow_bonus` (new, `ThrowAgent.cs`) pays at release up to 0.5 for the elbow's share of the ball's speed, full at a share of 0.25; negative shares (the elbow bending) earn nothing.
`far_elbow` is `far_squirt` with only that term added (`ml/config/far_elbow.yaml`): the same 8M-step fine-tune from `nat_squirt`, the same ramp from 70 to 92.

[![the same far target, both policies](media/far/elbow-vs-far.jpg)](media/far/elbow-vs-far.mp4)

[elbow-vs-far.mp4](media/far/elbow-vs-far.mp4): the same far held-out target (episode 4 of each policy's 120 s take on the far set), both hits, the same locked camera, 0.25× for 0.3 s around the release and 0.1 s before the hit.
The still is just after the release: `far_squirt` (left) has bent its elbow through the throw, `far_elbow` (right) has straightened it.

| | held-out, standard set | held-out, far set (76/82/88/92) | carry, far set (mean / max) | backswing (far) | share s/e/w/f (standard) | share s/e/w/f (far) |
|---|---|---|---|---|---|---|
| `nat_squirt` | 62% (72/80/70/19) | 11% (30/13/0/0) | 75.6 / 93.5 | 59° | 0.81 / −0.08 / 0.04 / 0.23 | 0.87 / −0.16 / −0.05 / 0.34 |
| `far_squirt` | 58% (69/70/86/2) | 12% (29/2/17/0) | 67.6 / 97.4 | 52° | 0.85 / −0.01 / 0.09 / 0.08 | 0.98 / −0.02 / 0.01 / 0.04 |
| `far_elbow` | **63%** (67/84/60/**40**) | **16%** (37/21/5/0) | 69.1 / 101.6 | 56° | 0.53 / **0.32** / 0.06 / 0.10 | 0.42 / **0.50** / −0.07 / 0.16 |

240 held-out throws per cell, the same eval as above.

The term changes the motion: the elbow, which bent through every earlier policy's release, now extends into it and gives a third (standard set) to a half (far set) of the ball's speed, the shoulder carrying most of the rest.
That is the proximal-to-distal hand-off of a human throw, with the wrist and fingers still adding little.
It also throws better: the 70-out band doubles (19% to 40% against `nat_squirt`), the far set rises from 12% to 16% (39 of 240 against 29), and the standard set is back to `nat_squirt`'s level (63% against 62%).
The 88 and 92 bands are still near zero, and the ramp again stalled short of them (78.3 at 8M, frontier throws hitting 15 to 25%).
The fingers' share rose a little from `far_squirt` (0.04 to 0.16 on the far set), still under `nat_squirt`'s.

Run: `tools/train/train.sh far_elbow --build --init nat_squirt --gpu --config far_elbow.yaml`, 3.7 h on the shared machine; checkpoint `ml/results/far_elbow/Throw/Throw-8000035.onnx` (not committed).

## Next

- A longer `far_elbow` run with the near half of each lesson dropped, so the training steps go to the frontier at 80 to 92.
- `far_elbow` as the policy for the site footage, if the PI likes how it throws: the elbow extension reads in a closer shot.
