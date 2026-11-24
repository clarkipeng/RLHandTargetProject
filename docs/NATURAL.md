# Making the throw look natural

The wind-up policy of round 2 (`windup13`, 40M steps, [WINDUP.md](WINDUP.md)) hits, but the PI's verdict was that it still does not move like a normal arm when it throws.
This round tries four changes (a, b, d and e; c was not tried), each a 6M-step fine-tune of `windup13` on the same held ramp, and measures them the same way.

[![four policies, the same throw](media/natural/grid.jpg)](media/natural/grid.mp4)

[grid.mp4](media/natural/grid.mp4): the same 70-out held-out throw (target 22, azimuth 30) by each policy, locked camera, static background, 0.25× from 1.35 s before the release.
Labels give each policy's held-out hit rate.

## What makes it look wrong

The measurement that explains the look is the release-speed share.
At the last step the hand touches the ball, the ball's velocity is split into what each link adds to it (the telescoping point velocities of the shoulder, elbow and palm links at the ball's centre); the "fingers" share is what is left, the ball's speed relative to the palm.
In `windup13` that share is **0.66**: two thirds of the ball's speed comes from the fingers squeezing it out of the hand, not from the arm carrying it.
The elbow's share is negative (−0.11): the elbow is bending, not extending, at release.
A human overarm throw is the reverse: the trunk and shoulder rotate, the elbow extends hard just before release, and the ball leaves the fingers moving with the hand.

## The variants

| | variant | motivation | held-out hits (26/40/52/70) | backswing | share s/e/w/f | LDLJ |
|---|---|---|---|---|---|---|
| | current (`windup13`, 40M) | the baseline | 65% (77/81/74/24) | 34° | 0.44 / −0.11 / 0.01 / 0.66 | −21.6 |
| d | smoother control | a stronger low-pass on the drive targets (`action_smoothing` 0.25 → 0.05, about an 80 ms first-order filter); step-to-step jitter reads as robotic | **77%** (89/97/77/40) | 35° | 0.71 / −0.22 / −0.05 / 0.55 | −21.8 |
| b | shaping | penalize action rate (10×) and effort (torque²), and pay at release for proximal-to-distal peak order | 72% (76/91/85/32) | 37° | 0.44 / −0.05 / −0.02 / 0.63 | −21.6 |
| a | human wrist | the wrist turns −180..90° and −90..90°, a human's about −80..70° and ±30°; fingers capped at 200 deg/s | 12% (39/8/0/0), stopped at 2M | 35° | 0.88 / −0.04 / −0.25 / 0.41 | −22.4 |
| e | no squirt | charge each throw its fingers share, so the speed has to come from the arm | 64% (67/83/78/22) | 38° | 0.77 / −0.09 / 0.10 / **0.22** | −21.8 |

240 held-out throws each, at 26/40/52/70 out in six azimuth bands, deterministic actions.
LDLJ is the log dimensionless jerk of the palm's path from the episode start to 0.3 s after the release (Hogan and Sternad; closer to zero is smoother), over a 40 s take.
The comparison chart of shares, peak speeds and accelerations is [joints.png](media/natural/joints.png).

### d: smoother control

The best hit rate, and the shoulder's share of the release rises from 0.44 to 0.71.
But it is unclear how much of the gain is the filter and how much is 6M more steps of training: the control run that would separate them was dropped for variant e (the machine was shared and training ran at 40% speed).
The jerk score did not move, and the elbow still bends at release.

### b: shaping

Hits and backswing barely move.
The proximal-to-distal bonus is paid on the order of each joint's peak speed, and the policy can collect most of it without changing what the ball sees: the shares are the same as the baseline's.

### a: human wrist

The narrower wrist and slower fingers take away the flick the policy relies on: throws carry 25 units on average (44 before) and it hit 12% after 2M steps, still learning (the backswing grew from 18° to 24° in training).
The fingers still peaked at 957 deg/s against the 200 cap, from the ball's contact pushing them (WINDUP.md, "Contacts and passive whip").
It needs a much longer run, from scratch, to be judged.

### c: a motion prior (not tried)

DeepMimic or AMP with a CMU mocap throw needs the demonstration in the agent's own observation and action space: the mocap retargeted by inverse kinematics onto this arm's 15 drives, its proportions and joint ranges, and the ball.
That retargeting is a project of its own; the variants above say what it would have to fix first.

### e: no squirt

The one variant that changes how the ball is thrown.
`squirt_cost 1` charges every training throw its fingers share at release (clamped to 0..1), so popping the ball out of the hand costs up to as much as a hit pays.
The fingers share falls from 0.66 to 0.22, the shoulder carries 0.77 and the wrist now adds 0.10, at the same hit rate (64% against 65%), the same throw distance (44 mean, 85 max) and a slightly longer backswing (38°).
The fingers still move as fast as before (529 deg/s peak); they no longer push the ball.
It is slow to take: at 1M steps the share was still 0.61, so most of the change came in the last 5M.
The elbow still bends at release (−0.09), and the jerk score is unchanged.

## Which to keep

**Keep e (no squirt), and train it further with d's filter.**
The PI's complaint is about how the arm throws, and e is the only change that moves the ball's speed from the fingers to the arm without losing hits.
d's hit rate is the best but it leaves the squeeze (0.55) in place, and its gain is not separated from the extra training.
b changed nothing measurable, and a needs a long run of its own.

What is still not human, in order of how much it shows:

1. **The elbow does not extend at release** in any variant (−0.05 to −0.22). A human throw gets much of its speed from the elbow straightening in the last 50 ms. A next step is a release-speed reward on the elbow's share, or a start pose with the elbow bent so it has room to extend.
2. **No proximal-to-distal timing.** Peak speeds come in no consistent order (the wrist often peaks before the shoulder); the sequence bonus in b did not change it.
3. **The jerk score is flat** across all variants (−21.6 to −22.4), so the palm path is no smoother; the look of smoothness is not where the policies differ.

## Showcase look

Every showcase shot now has a locked camera (`move` false, the default since PR #10), and the live view no longer drifts.
The live view and these clips are back on the Art look, the hall with light fins and bullseye targets that give the throw a scale; the Top light is brighter (0.35 → 1.1) and the ambient lifted, because the fixed key light misses the arm on throws to the side.
`?look=minimal` on the live view shows the studio look, and `?look=cycle` alternates.
