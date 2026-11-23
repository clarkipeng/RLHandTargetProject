"""How far can the capped arm throw, with and without a wind-up?

Runs the player in probe mode (probe=1): the ball starts fixed to the hand, nothing is released, and every physics step
records how far the ball would carry (drag-free) if it were let go at that instant; the episode reports the best.
This script searches open-loop swings for the shoulder, elbow and wrist (the fingers keep the start pose) with the
cross-entropy method. A swing is two targets, A1 held for T seconds and then A2. Two searches:

  direct:  the measured backswing (the hand's turn away from the throw before it) must stay under --direct-backswing
           degrees, so the swing goes straight at the target from the start pose;
  windup:  unconstrained, so it may first swing back.

If the farthest direct carry already covers every target, there is no reason for a policy to wind up.

  uv run python probe.py --tag rest                       # PR #8 start pose
  uv run python probe.py --pose 1=-0.6,3=0.2 --tag low    # pose_<i> in action units
  uv run python probe.py --pose-only --pose 1=-0.6        # log where the hand and pedestal would be, then exit
"""
import argparse, json, os, re, time
from pathlib import Path

import numpy as np
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.environment_parameters_channel import EnvironmentParametersChannel
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel

HERE = Path(__file__).resolve().parent
SHIM = HERE.parent / "tools" / "train" / "player"
GROUPS = ["shoulder", "elbow", "wrist", "fingers"]
SWING = 6           # drives 0-5: shoulder x/y/z, elbow, wrist x/y
DT = 1 / 30         # one decision
CAPS = {"shoulder": 540, "elbow": 540, "wrist": 300}   # deg/s, ArmLimits.Speed
LINE = re.compile(r"\[probe\] arena (\d+) carry (\S+) high (\S+) speed (\S+) at (\S+) backswing (\S+) windup (\S+) fwd (\S+) "
                  r"share (\S+) speed (\S+) over (\S+) act (\S+)(?: drift (\S+))?(?: glitch (\S+))?")
POSE = re.compile(r"\[pose\] grip (\S+) (\S+) (\S+) palm (\S+) (\S+) (\S+)")


def vec(s): return [float(x) for x in s.split(",")]


def parse(text):
    rows = []
    for m in LINE.finditer(text):
        g = m.groups()
        rows.append({"carry": float(g[1]), "high": float(g[2]), "speed": float(g[3]), "at": float(g[4]),
                     "backswing": float(g[5]), "windup": float(g[6]), "fwd": float(g[7]),
                     "share": dict(zip(GROUPS, vec(g[8]))), "peak": dict(zip(GROUPS, vec(g[9]))),
                     "over": dict(zip(GROUPS, vec(g[10]))), "act": vec(g[11]), "drift": float(g[12] or -1), "glitch": int(g[13] or -1)})
    return rows


class Player:
    def __init__(self, a, extra):
        os.environ.setdefault("THROW_PLAYER", "/mnt/c/agentworld/rlhand/Builds/Throw/Throw.exe")
        os.environ["THROW_ARENAS"] = str(a.arenas)
        self.params, engine = EnvironmentParametersChannel(), EngineConfigurationChannel()
        self.log = HERE / "results" / f"probe_player_{a.port}.log"
        if self.log.exists(): self.log.unlink()
        self.env = UnityEnvironment(str(SHIM), base_port=a.port, side_channels=[self.params, engine], no_graphics=True,
                                    additional_args=["-logFile", str(self.log)], timeout_wait=120)
        engine.set_configuration_parameters(time_scale=50.0, target_frame_rate=-1, capture_frame_rate=0)
        for k, v in {"probe": 1, "probe_seconds": a.seconds, "arm_limits": 1, "eval_target": -2, "lesson": 5,
                     **extra}.items():
            self.params.set_float_parameter(k, float(v))
        self.env.reset()
        self.name = list(self.env.behavior_specs)[0]
        self.read = 0

    def text(self):
        t = self.log.read_text(errors="ignore") if self.log.exists() else ""
        return t

    def run(self, plans, rest):
        """plans: list of (A1, T, A2). Returns one result row per plan (matched by the final action the agent logs)."""
        todo = list(range(len(plans)))
        current, finished = {}, 0
        phase = getattr(self, "phase", {})          # decisions since each agent's episode began
        while finished < len(plans):
            dec, term = self.env.get_steps(self.name)
            for aid in term.agent_id:
                phase[aid] = 0
                if aid in current: current.pop(aid); finished += 1
            if len(dec) > 0:
                acts = np.tile(rest, (len(dec), 1)).astype(np.float32)
                for j, aid in enumerate(dec.agent_id):
                    ph = phase.get(aid, 0)
                    if aid not in current and ph == 0 and todo: current[aid] = todo.pop(0)
                    if aid in current:
                        a1, T, a2 = plans[current[aid]]
                        acts[j, :SWING] = a1 if ph * DT < T else a2
                    phase[aid] = ph + 1
                self.env.set_actions(self.name, ActionTuple(continuous=acts))
            self.env.step()
        self.phase = phase
        # Match the logged rows to plans by the final swing targets (the log can lag the episode end a little).
        out = [None] * len(plans)
        for _ in range(20):
            rows = parse(self.text())[self.read:]
            used = set()
            for k, (a1, T, a2) in enumerate(plans):
                for n, r in enumerate(rows):
                    if n not in used and np.allclose(r["act"][:SWING], a2, atol=2e-4):
                        out[k] = r; used.add(n); break
            if all(o is not None for o in out): break
            time.sleep(0.25)
        self.read += len(rows)
        return out

    def close(self):
        try: self.env.close()
        except Exception: pass


def cem(player, rest, direct, a, rng):
    dim = 2 * SWING + 1
    mu = np.concatenate([rest[:SWING], [0.3], rest[:SWING]]).astype(float)
    sd = np.concatenate([np.full(SWING, 0.6), [0.25], np.full(SWING, 0.6)])
    best = (-1e9, None, None)
    history = []
    for g in range(a.gens):
        X = mu + sd * rng.standard_normal((a.pop, dim))
        if g == 0 and direct:
            X[0] = mu  # the start pose held: carry from simply holding still
        X[:, :SWING] = np.clip(X[:, :SWING], -1, 1); X[:, SWING + 1:] = np.clip(X[:, SWING + 1:], -1, 1)
        X[:, SWING] = np.clip(X[:, SWING], 0.0, 1.2)
        X = np.round(X, 4)
        plans = [(x[:SWING], float(x[SWING]), x[SWING + 1:]) for x in X]
        rows = player.run(plans, rest)
        fit = []
        for x, r in zip(X, rows):
            if r is None: fit.append(-1e9); continue
            f = r["carry"]
            # A swing whose arm joints ran well past their caps blew up the physics (see the README of this probe): no score.
            if any(r["peak"][g] > 1.2 * c for g, c in CAPS.items()): f = -1.0
            if direct: f -= 10 * max(0.0, r["backswing"] - a.direct_backswing)
            fit.append(f)
            if f > best[0]: best = (f, x.tolist(), r)
        fit = np.array(fit)
        elite = X[np.argsort(-fit)[:a.elite]]
        mu = 0.3 * mu + 0.7 * elite.mean(0)
        sd = np.maximum(0.7 * sd + 0.3 * elite.std(0), 0.03)
        b = best[2]
        history.append({"gen": g, "best": round(best[0], 2), "gen_best": round(float(fit.max()), 2),
                        "matched": int(sum(r is not None for r in rows))})
        print(f"  {'direct' if direct else 'windup'} gen {g:2d}  gen best {fit.max():6.2f}  best {best[0]:6.2f}  "
              f"(drift {b['drift']:.2f} carry {b['carry']:.1f} backswing {b['backswing']:.1f} speed {b['speed']:.1f})" if b else "", flush=True)
    return best, history


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", default="", help="i=v,... start pose in action units (pose_<i>)")
    ap.add_argument("--set", action="append", default=[], metavar="K=V", help="any other environment parameter")
    ap.add_argument("--arenas", type=int, default=8)
    ap.add_argument("--port", type=int, default=6300)
    ap.add_argument("--seconds", type=float, default=2.5)
    ap.add_argument("--gens", type=int, default=20)
    ap.add_argument("--pop", type=int, default=48)
    ap.add_argument("--elite", type=int, default=8)
    ap.add_argument("--direct-backswing", type=float, default=5.0)
    ap.add_argument("--mode", choices=["both", "direct", "windup"], default="both")
    ap.add_argument("--pose-only", action="store_true")
    ap.add_argument("--replay", help="JSON file with one plan [A1(6), T, A2(6)]: run it once with a per-step [ptrace] and print it")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tag", default="")
    ap.add_argument("--out", default=str(HERE / "results" / "windup2" / "probe.jsonl"))
    a = ap.parse_args()

    extra = {}
    pose = {}
    for kv in filter(None, a.pose.split(",")):
        i, v = kv.split("="); pose[int(i)] = float(v); extra[f"pose_{int(i)}"] = float(v)
    for kv in a.set:
        k, v = kv.split("="); extra[k] = float(v)
    # Start pose in action units, as the agent computes it (the elbow and fingers rest at their upper limit, +1).
    rest = np.array([-0.5, 0, 0, 1, 0, 0] + [1] * 9, dtype=np.float32)   # ThrowAgent.DefaultPose: shoulder 45 degrees forward
    for i, v in pose.items(): rest[i] = v

    player = Player(a, extra)
    player.seconds = a.seconds
    rng = np.random.default_rng(a.seed)
    t0 = time.time()
    try:
        if a.replay:
            x = json.load(open(a.replay))
            r = player.run([(np.array(x[:SWING]), float(x[SWING]), np.array(x[SWING + 1:]))], rest)[0]
            print("\n".join(l for l in player.text().splitlines() if l.startswith("[ptrace]")))
            print(r)
            return 0
        hold = player.run([(rest[:SWING], 0.0, rest[:SWING])], rest)[0]
        m = POSE.search(player.text())
        where = {"grip": [float(x) for x in m.groups()[:3]], "palm": [float(x) for x in m.groups()[3:]]} if m else None
        print(f"start pose {pose or 'rest'}: {where}; holding still carries {hold and hold['carry']:.1f}")
        row = {"tag": a.tag, "pose": pose, "set": a.set, "where": where, "seconds": a.seconds,
               "hold_carry": hold and hold["carry"], "time": time.strftime("%Y-%m-%dT%H:%M:%S")}
        if a.replay:
            x = json.load(open(a.replay))
            r = player.run([(np.array(x[:SWING]), float(x[SWING]), np.array(x[SWING + 1:]))], rest)[0]
            print("\n".join(l for l in player.text().splitlines() if l.startswith("[ptrace]")))
            print(r)
            return 0
        if not a.pose_only:
            for mode in (["direct", "windup"] if a.mode == "both" else [a.mode]):
                (f, x, r), hist = cem(player, rest, mode == "direct", a, rng)
                row[mode] = {"fitness": f, "plan": x, "result": r, "history": hist}
    finally:
        player.close()
    row["wall_seconds"] = round(time.time() - t0, 1)
    with open(a.out, "a") as fh: fh.write(json.dumps(row) + "\n")
    for mode in ("direct", "windup"):
        if mode in row and row[mode]["result"]:
            r = row[mode]["result"]
            print(f"{mode}: carry {r['carry']:.1f} (high target {r['high']:.1f}) speed {r['speed']:.1f} backswing {r['backswing']:.1f} "
                  f"drift {r['drift']} peak {r['peak']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
