"""Held-out hit rate of one exported policy.

    uv run python eval.py results/<run>/Throw/Throw-<step>.onnx [--episodes 48] [--arenas 4] [--take out.take]

Runs the Windows training player with eval_target=-2: every arena cycles through the 24 held-out targets
(azimuths never used in training), deterministic actions, seeded ball placement.
A hit pays +1.5 at the end of an episode, everything else at most 0.8, so the episode return tells hits apart.
Appends one JSON line per checkpoint to results/<run>/eval.jsonl (or --out) and prints it: the hit rate overall and per
held-out distance (25, 33, 42, 52), and per-throw means from the player's [eval] log lines (throw distance, release
speed, each joint group's share of the release speed, peak joint speed and acceleration, backswing and wind-up time).
--limits old runs the arm of PR #6 (no torque or speed caps); the default is the capped arm (ArmLimits in ThrowAgent.cs).
--set k=v sets any other environment parameter (e.g. torque_wrist=100 for a strength sweep).
With --take, the player also records the rollout for the replay renderer (one arena).
"""
import argparse, json, os, re, sys, time
from pathlib import Path

import numpy as np
import onnxruntime as ort
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.environment_parameters_channel import EnvironmentParametersChannel
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

HERE = Path(__file__).resolve().parent
SHIM = HERE.parent / "tools" / "train" / "player"
HIT_RETURN = 1.2
GROUPS = ["shoulder", "elbow", "wrist", "fingers"]
LINE = re.compile(r"\[eval\] target (\d+) hit (\d) grasp (\d) lift (\d) throw (\d) gap (\S+) dist (\S+) thrown (\S+) vrel (\S+) "
                  r"backswing (\S+) windup (\S+) share (\S+) speed (\S+) accel (\S+) torque (\S+)"
                  r"(?: pkc (\S+) pkp (\S+) pkt (\S+) over (\S+) fwd (\S+))?(?: sspeed (\S+))?(?: wspeed (\S+))?(?: gov (\S+))?")


def parse_log(path):
    """One dict per [eval] episode line in the player log."""
    rows = []
    for line in open(path, errors="replace"):
        m = LINE.search(line)
        if not m: continue
        g = m.groups()
        vec = lambda s: [float(x) for x in s.split(",")]
        rows.append(dict(target=int(g[0]), hit=int(g[1]), grasp=int(g[2]), lift=int(g[3]), throw=int(g[4]), gap=float(g[5]),
                         dist=float(g[6]), thrown=float(g[7]), vrel=float(g[8]), backswing=float(g[9]), windup=float(g[10]),
                         share=vec(g[11]), speed=vec(g[12]), accel=vec(g[13]), torque=vec(g[14])))
        if g[15] is not None:  # peak context (contact, drive pushing, steps before release), clamped steps, forward swing
            rows[-1].update(pkc=vec(g[15]), pkp=vec(g[16]), pkt=vec(g[17]), over=vec(g[18]), fwd=float(g[19]))
        if g[20] is not None:  # peak speed over 1/60 s (the angle covered), next to the one-step peak
            rows[-1]["sspeed"] = vec(g[20])
        if g[21] is not None:  # peak speed in the throw: from the backswing's turning point to the release
            rows[-1]["wspeed"] = vec(g[21])
        if g[22] is not None:  # physics steps the speed governor held each group
            rows[-1]["gov"] = vec(g[22])
    return rows


def summarize(rows):
    """Hit rate per held-out distance, and per-throw means (thrown episodes only) of the throw measurements."""
    out = {"bands": {}}
    for d in sorted({r["dist"] for r in rows}):
        b = [r for r in rows if r["dist"] == d]
        out["bands"][f"{d:g}"] = {"hit_rate": sum(r["hit"] for r in b) / len(b), "episodes": len(b),
                                  "thrown": float(np.mean([r["thrown"] for r in b if r["throw"]] or [0]))}
    if rows:
        out.update({k + "_rate": sum(r[k] for r in rows) / len(rows) for k in ("grasp", "lift", "throw")})
    t = [r for r in rows if r["throw"]]
    if t:
        mean = lambda k: float(np.mean([r[k] for r in t]))
        vmean = lambda k: {n: float(np.mean([r[k][i] for r in t])) for i, n in enumerate(GROUPS)}
        vmax = lambda k: {n: float(np.percentile([r[k][i] for r in t], 90)) for i, n in enumerate(GROUPS)}
        out.update(throws=len(t), thrown=mean("thrown"), thrown_max=float(max(r["thrown"] for r in t)), release_speed=mean("vrel"),
                   backswing=mean("backswing"), windup=mean("windup"), share=vmean("share"), peak_speed=vmean("speed"),
                   peak_accel=vmean("accel"), peak_torque_p90=vmax("torque"),
                   peak_speed_max={n: float(max(r["speed"][i] for r in t)) for i, n in enumerate(GROUPS)},
                   backswing_p90=float(np.percentile([r["backswing"] for r in t], 90)))
        if "pkc" in t[0]:
            out.update(peak_in_contact=vmean("pkc"), peak_drive_pushing=vmean("pkp"), peak_frames_after_release=vmean("pkt"),
                       clamped_steps=vmean("over"), forward_swing=mean("fwd"))
        if "sspeed" in t[0]:
            if "gov" in t[0]:
                out["governed_steps"] = vmean("gov")
            if "wspeed" in t[0]:
                out.update(throw_peak=vmean("wspeed"), throw_peak_max={n: float(max(r["wspeed"][i] for r in t)) for i, n in enumerate(GROUPS)},
                           throw_peak_p90=vmax("wspeed"))
            out.update(peak_sustained=vmean("sspeed"), peak_sustained_max={n: float(max(r["sspeed"][i] for r in t)) for i, n in enumerate(GROUPS)})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("onnx")
    ap.add_argument("--episodes", type=int, default=48)
    ap.add_argument("--arenas", type=int, default=4)
    ap.add_argument("--take", help="also record a .take of the rollout (forces one arena)")
    ap.add_argument("--seconds", type=float, default=60.0, help="take length")
    ap.add_argument("--port", type=int, default=6100)
    ap.add_argument("--motion", type=float, default=0.0, help="sway every target sideways by this many units (showcase; never trained)")
    ap.add_argument("--period", type=float, default=6.0, help="seconds per sway with --motion")
    ap.add_argument("--lesson", type=int, help="with --take: show this curriculum lesson's training targets instead of the held-out set")
    ap.add_argument("--limits", choices=["new", "old"], default="new", help="old: the uncapped arm of PR #6")
    ap.add_argument("--set", action="append", default=[], metavar="K=V", help="any other environment parameter")
    ap.add_argument("--out", help="append the row here instead of results/<run>/eval.jsonl")
    ap.add_argument("--tag", help="label stored in the row")
    a = ap.parse_args()

    onnx = Path(a.onnx).resolve()
    sess = ort.InferenceSession(str(onnx), providers=["CPUExecutionProvider"])
    outs = [o.name for o in sess.get_outputs()]
    out = "deterministic_continuous_actions" if "deterministic_continuous_actions" in outs else "continuous_actions"
    inp = sess.get_inputs()[0].name

    args = []
    os.environ.setdefault("THROW_PLAYER", "/mnt/c/agentworld/rlhand/Builds/Throw/Throw.exe")
    os.environ["THROW_ARENAS"] = "1" if a.take else str(a.arenas)
    if a.take:
        take = Path(a.take).resolve()
        args += ["-showcaseTake", str(take), "-showcaseSeconds", str(a.seconds), "-showcasePolicy", onnx.stem]
    params, engine = EnvironmentParametersChannel(), EngineConfigurationChannel()
    log = HERE / "results" / f"eval_player_{a.port}.log"
    if log.exists(): log.unlink()
    env = UnityEnvironment(str(SHIM), base_port=a.port, side_channels=[params, engine], no_graphics=True,
                           additional_args=args + ["-logFile", str(log)], timeout_wait=120)
    engine.set_configuration_parameters(time_scale=20.0 if a.take else 50.0, target_frame_rate=-1, capture_frame_rate=0)
    show_lesson = a.take and a.lesson is not None
    params.set_float_parameter("eval_target", -1.0 if show_lesson else -2.0)
    params.set_float_parameter("lesson", float(a.lesson) if show_lesson else 5.0)
    params.set_float_parameter("target_motion", a.motion)
    params.set_float_parameter("target_period", a.period)
    params.set_float_parameter("arm_limits", 1.0 if a.limits == "new" else 0.0)
    for kv in a.set:
        k, v = kv.split("=")
        params.set_float_parameter(k, float(v))
    env.reset()
    name = list(env.behavior_specs)[0]
    returns, running = [], {}
    t0 = time.time()
    want = a.episodes
    if a.take:
        want = 10**9  # record until the player has written the take and quit
    try:
        while len(returns) < want:
            dec, term = env.get_steps(name)
            for i, aid in enumerate(term.agent_id):
                returns.append(running.pop(aid, 0.0) + float(term.reward[i]))
            for i, aid in enumerate(dec.agent_id):
                running[aid] = running.get(aid, 0.0) + float(dec.reward[i])
            if len(dec) > 0:
                act = sess.run([out], {inp: dec.obs[0].astype(np.float32)})[0]
                env.set_actions(name, ActionTuple(continuous=np.clip(act, -1, 1)))
            env.step()
    except Exception as e:
        if not a.take: raise
        print(f"player finished the take ({type(e).__name__})")
    finally:
        try: env.close()
        except Exception: pass
    if a.take:
        print(json.dumps({"take": str(take), "episodes": len(returns), "hits": int((np.array(returns) > HIT_RETURN).sum())}))
        return 0
    returns = np.array(returns[:want])
    hits = int((returns > HIT_RETURN).sum())
    m = re.search(r"-(\d+)\.onnx$", onnx.name)
    row = {"checkpoint": onnx.name, "step": int(m.group(1)) if m else None, "episodes": len(returns),
           "hits": hits, "hit_rate": hits / len(returns), "mean_return": float(returns.mean()),
           "seconds": round(time.time() - t0, 1), "time": time.strftime("%Y-%m-%dT%H:%M:%S"),
           "limits": a.limits, "set": a.set, "tag": a.tag}
    if log.exists():
        row.update(summarize(parse_log(log)[:want]))
    if not a.take:
        with open(Path(a.out) if a.out else onnx.parent.parent / "eval.jsonl", "a") as f:
            f.write(json.dumps(row) + "\n")
    print(json.dumps(row))


if __name__ == "__main__":
    sys.exit(main())
