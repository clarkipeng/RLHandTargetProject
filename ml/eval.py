"""Held-out hit rate of one exported policy.

    uv run python eval.py results/<run>/Throw/Throw-<step>.onnx [--episodes 48] [--arenas 4] [--take out.take]

Runs the Windows training player with eval_target=-2: every arena cycles through the 24 held-out targets
(azimuths never used in training), deterministic actions, seeded ball placement.
A hit pays +1.5 at the end of an episode, everything else at most 0.8, so the episode return tells hits apart.
Appends one JSON line per checkpoint to results/<run>/eval.jsonl and prints it.
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("onnx")
    ap.add_argument("--episodes", type=int, default=48)
    ap.add_argument("--arenas", type=int, default=4)
    ap.add_argument("--take", help="also record a .take of the rollout (forces one arena)")
    ap.add_argument("--seconds", type=float, default=60.0, help="take length")
    ap.add_argument("--port", type=int, default=6100)
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
    log = HERE / "results" / "eval_player.log"
    env = UnityEnvironment(str(SHIM), base_port=a.port, side_channels=[params, engine], no_graphics=True,
                           additional_args=args + ["-logFile", str(log)], timeout_wait=120)
    engine.set_configuration_parameters(time_scale=20.0 if a.take else 50.0, target_frame_rate=-1, capture_frame_rate=0)
    params.set_float_parameter("eval_target", -2.0)
    params.set_float_parameter("lesson", 5.0)
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
           "seconds": round(time.time() - t0, 1), "time": time.strftime("%Y-%m-%dT%H:%M:%S")}
    if not a.take:
        with open(onnx.parent.parent / "eval.jsonl", "a") as f:
            f.write(json.dumps(row) + "\n")
    print(json.dumps(row))


if __name__ == "__main__":
    sys.exit(main())
