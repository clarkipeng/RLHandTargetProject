"""start_held check: the hand closed on the ball, the arm held still, then (optionally) one scripted swing.
Prints the [eval] row of each episode: does the ball stay in the hand, and how far does a scripted swing throw it?
  uv run python holdtest.py '<plan json: list of [A1(6), T, A2(6)]>' [--take out.take] [--set k=v]"""
import os, sys, re, json, argparse, numpy as np
from pathlib import Path
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.environment_parameters_channel import EnvironmentParametersChannel
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser(); ap.add_argument("plans"); ap.add_argument("--take"); ap.add_argument("--set", action="append", default=[])
ap.add_argument("--port", type=int, default=6500); ap.add_argument("--fingers", type=float, default=-1.0)
a = ap.parse_args(); plans = json.loads(a.plans)
os.environ.setdefault("THROW_PLAYER", "/mnt/c/agentworld/rlhand/Builds/Throw/Throw.exe"); os.environ["THROW_ARENAS"] = "1"
args = ["-showcaseTake", str(Path(a.take).resolve()), "-showcaseSeconds", str(10.5 * len(plans)), "-showcasePolicy", "scripted"] if a.take else []
params, eng = EnvironmentParametersChannel(), EngineConfigurationChannel()
log = HERE / "results" / f"holdtest_{a.port}.log"
if log.exists(): log.unlink()
env = UnityEnvironment(str(HERE.parent / "tools/train/player"), base_port=a.port, side_channels=[params, eng], no_graphics=True,
                       additional_args=args + ["-logFile", str(log)], timeout_wait=120)
eng.set_configuration_parameters(time_scale=20.0 if a.take else 50.0, target_frame_rate=-1, capture_frame_rate=0)
kv = {"eval_target": -2, "lesson": 5, "arm_limits": 1, "start_held": 1, "ped_y": -14.5, "episode_seconds": 4}
for s in a.set: k, v = s.split("="); kv[k] = float(v)
for k, v in kv.items(): params.set_float_parameter(k, float(v))
env.reset(); name = list(env.behavior_specs)[0]
rest = np.array([-0.5, 0, 0, 1, 0, 0] + [a.fingers] * 9, dtype=np.float32)
done, step, k = 0, 0, 0
try:
    while done < len(plans):
        dec, term = env.get_steps(name)
        if len(term): done += 1; step = 0; k = min(done, len(plans) - 1)
        if len(dec):
            act = rest.copy()[None].repeat(len(dec), 0)
            p = plans[k]
            if p:
                act[:, :6] = p[:6] if step / 30 < 0.6 + p[6] else p[7:13]
                if step / 30 < 0.6: act[:, :6] = rest[:6]
                if len(p) > 13 and step / 30 > 0.6 + p[6] + p[13]: act[:, 6:] = 1.0   # open the hand: release
            env.set_actions(name, ActionTuple(continuous=act)); step += 1
        env.step()
except Exception as e:
    print("ended:", type(e).__name__)
finally:
    try: env.close()
    except Exception: pass
for l in re.findall(r"\[eval\] (.*?) share", log.read_text(errors="ignore")): print(l)
