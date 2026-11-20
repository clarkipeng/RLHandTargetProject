"""Smoke test of the training player: random actions in each lesson; prints episode counts, returns, lengths and speed.
    uv run python smoke.py [--arenas 8] [--seconds 20]"""
import argparse, os, time
from pathlib import Path
import numpy as np
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.environment_parameters_channel import EnvironmentParametersChannel
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

HERE = Path(__file__).resolve().parent
ap = argparse.ArgumentParser(); ap.add_argument("--arenas", type=int, default=8); ap.add_argument("--seconds", type=float, default=20)
ap.add_argument("--lessons", default="0,2,5"); a = ap.parse_args()
os.environ.setdefault("THROW_PLAYER", "/mnt/c/agentworld/rlhand/Builds/Throw/Throw.exe")
os.environ["THROW_ARENAS"] = str(a.arenas)
params, engine = EnvironmentParametersChannel(), EngineConfigurationChannel()
env = UnityEnvironment(str(HERE.parent / "tools/train/player"), base_port=6200, side_channels=[params, engine], no_graphics=True,
                       additional_args=["-logFile", str(HERE / "results/smoke_player.log")], timeout_wait=120)
engine.set_configuration_parameters(time_scale=100, target_frame_rate=-1, capture_frame_rate=0)
env.reset()
name = list(env.behavior_specs)[0]; spec = env.behavior_specs[name]
print("behavior", name, "obs", [o.shape for o in spec.observation_specs], "actions", spec.action_spec.continuous_size)
for lesson in [int(x) for x in a.lessons.split(",")]:
    params.set_float_parameter("lesson", float(lesson))
    rets, lens, run, ln = [], [], {}, {}
    t0, steps = time.time(), 0
    while time.time() - t0 < a.seconds:
        dec, term = env.get_steps(name)
        for i, aid in enumerate(term.agent_id):
            rets.append(run.pop(aid, 0) + term.reward[i]); lens.append(ln.pop(aid, 0) + 1)
        for i, aid in enumerate(dec.agent_id):
            run[aid] = run.get(aid, 0) + dec.reward[i]; ln[aid] = ln.get(aid, 0) + 1
        obs = dec.obs[0] if len(dec) else None
        if obs is not None and not np.isfinite(obs).all(): print("NON-FINITE OBS")
        if len(dec): env.set_actions(name, ActionTuple(continuous=np.random.uniform(-1, 1, (len(dec), spec.action_spec.continuous_size)).astype(np.float32)))
        env.step(); steps += len(dec)
    dt = time.time() - t0
    print(f"lesson {lesson}: {len(rets)} episodes, return mean {np.mean(rets) if rets else float('nan'):.3f} max {np.max(rets) if rets else float('nan'):.3f}, "
          f"len mean {np.mean(lens) if lens else 0:.0f} decisions, {steps/dt:.0f} decisions/s")
env.close()
