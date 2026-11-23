"""start_held: which finger closure and wrist pose keep the ball in a still hand? One player, a grid of settings,
two episodes each (after one discarded while the setting takes effect); the arm holds its start pose throughout."""
import os, sys, re, json, itertools, numpy as np
from pathlib import Path
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.environment_parameters_channel import EnvironmentParametersChannel
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
HERE = Path(__file__).resolve().parent
grid = json.loads(sys.argv[1])        # list of {held_close, pose_4, pose_5, ...}
os.environ.setdefault("THROW_PLAYER", "/mnt/c/agentworld/rlhand/Builds/Throw/Throw.exe"); os.environ["THROW_ARENAS"] = "1"
params, eng = EnvironmentParametersChannel(), EngineConfigurationChannel()
log = HERE / "results" / "holdgrid.log"
if log.exists(): log.unlink()
env = UnityEnvironment(str(HERE.parent / "tools/train/player"), base_port=6550, side_channels=[params, eng], no_graphics=True,
                       additional_args=["-logFile", str(log)], timeout_wait=120)
eng.set_configuration_parameters(time_scale=50.0, target_frame_rate=-1, capture_frame_rate=0)
base = {"eval_target": -2, "lesson": 5, "arm_limits": 1, "start_held": 1, "ped_y": -14.5, "episode_seconds": 3}
for k, v in base.items(): params.set_float_parameter(k, float(v))
env.reset(); name = list(env.behavior_specs)[0]
def n_eval(): return len(re.findall(r"\[eval\]", log.read_text(errors="ignore"))) if log.exists() else 0
for g in grid:
    for k, v in g.items(): params.set_float_parameter(k, float(v))
    rest = np.array([-0.5, 0, 0, 1, g.get("pose_4", 0), g.get("pose_5", 0)] + [g.get("held_close", -1)] * 9, dtype=np.float32)
    start = n_eval(); ends = 0
    while ends < 3:
        dec, term = env.get_steps(name)
        ends += len(term)
        if len(dec): env.set_actions(name, ActionTuple(continuous=rest[None].repeat(len(dec), 0)))
        env.step()
    rows = re.findall(r"\[eval\].*? grasp (\d) lift (\d)", log.read_text(errors="ignore"))
    print(json.dumps(g), "grasp,lift of the last two:", rows[-2:], flush=True)
env.close()
