"""Read a showcase take (Assets/Showcase/Runtime/Take.cs) and score how smoothly the arm moves.

    uv run python take.py <file.take> [--part palm]

Per episode, over the throw (episode start to 0.3 s after the release), the hand path's log dimensionless jerk
(Hogan & Sternad 2009): -ln(T^5 / L^2 * integral |d3x/dt3|^2 dt), with T the window and L the path length,
from the palm's position sampled at every physics step and low-passed at 15 Hz. Higher (less negative) is smoother.
The release is where the ball, after riding in the hand for 0.1 s, gets more than 3 units off the palm.
"""
import argparse, json, struct, sys
import numpy as np

KINDS = {0: "arm", 1: "ball", 2: "target", 3: "prop"}


class R:
    def __init__(self, b): self.b, self.i = b, 0
    def take(self, n): v = self.b[self.i:self.i + n]; self.i += n; return v
    def i32(self): return struct.unpack("<i", self.take(4))[0]
    def f32(self): return struct.unpack("<f", self.take(4))[0]
    def u8(self): return self.take(1)[0]
    def str(self):
        n = s = 0
        while True:
            c = self.u8(); n |= (c & 0x7F) << s; s += 7
            if c < 0x80: break
        return self.take(n).decode()
    def ints(self): return [self.i32() for _ in range(self.i32())]


def load(path):
    r = R(open(path, "rb").read())
    assert r.str() == "SHOWTAKE" and r.i32() == 1
    t = {"step": r.f32(), "steps": r.i32(), "policy": r.str(), "episodes": r.ints(), "hits": r.ints(), "actors": []}
    for _ in range(r.i32()):
        a = {"id": r.i32(), "kind": KINDS[r.u8()]}
        a["parts"] = [r.str() for _ in range(r.i32())]
        a["birth"], a["death"], a["hit"] = r.i32(), r.i32(), r.u8() != 0
        n, p = r.i32(), len(a["parts"])
        raw = np.frombuffer(r.take(n * p * 40), dtype="<f4").reshape(n, p, 10)
        a["pos"], a["rot"] = raw[:, :, 0:3], raw[:, :, 3:7]
        t["actors"].append(a)
    return t


def lowpass(x, dt, hz=15.0):
    k = int(round(1 / (hz * dt)))  # box filter twice, ~hz cutoff
    if k < 2: return x
    w = np.ones(k) / k
    for _ in range(2): x = np.stack([np.convolve(x[:, j], w, mode="same") for j in range(x.shape[1])], 1)
    return x


def ldlj(x, dt):
    if len(x) < 20: return float("nan")
    v = np.gradient(x, dt, axis=0); j = np.gradient(np.gradient(v, dt, axis=0), dt, axis=0)
    T = len(x) * dt; L = np.sum(np.linalg.norm(np.diff(x, axis=0), axis=1))
    if L < 1e-6: return float("nan")
    return float(-np.log(T ** 5 / L ** 2 * np.sum(np.sum(j * j, 1)) * dt))


def episodes(t, part="palm"):
    arm = next(a for a in t["actors"] if a["kind"] == "arm")
    pi = next(i for i, p in enumerate(arm["parts"]) if p.split("/")[-1] == part)
    balls = [a for a in t["actors"] if a["kind"] == "ball"]
    hand = arm["pos"][:, pi, :]
    starts = sorted(t["episodes"]) + [t["steps"]]
    out = []
    for s, e in zip(starts[:-1], starts[1:]):
        a, b = s - arm["birth"] + 8, min(e - arm["birth"], len(hand)) - 2   # skip the reset steps
        if b - a < 40: continue
        # The release: the ball leaves the hand (more than 3 units off the palm) after riding in it for 0.1 s.
        rel = None
        for ball in balls:
            bs, be = max(s, ball["birth"]) - ball["birth"], min(e, ball["birth"] + len(ball["pos"])) - ball["birth"]
            if be - bs < 10: continue
            bp = ball["pos"][:, 0, :]
            held = 0
            for k in range(bs, be):
                hk = k + ball["birth"] - arm["birth"]
                if not (0 <= hk < len(hand)): continue
                d = np.linalg.norm(bp[k] - hand[hk])
                if d < 2.5: held += 1
                elif d > 3.0 and held >= 24 and a <= hk < b: rel = hk; break
            if rel: break
        end = min(b, (rel + int(0.3 / t["step"])) if rel else b)
        x = lowpass(hand[a:end].astype(float), t["step"])
        out.append({"start": round(s * t["step"], 2), "release": round(rel * t["step"], 2) if rel else None,
                    "ldlj": round(ldlj(x[5:-5], t["step"]), 2)})
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("take"); ap.add_argument("--part", default="palm")
    a = ap.parse_args()
    t = load(a.take)
    arm = next(x for x in t["actors"] if x["kind"] == "arm")
    print(f"{t['policy']}: {t['steps']} steps of {t['step']*1000:.2f} ms, {len(t['episodes'])} episodes, {len(t['hits'])} hits, arm parts {[p.split('/')[-1] for p in arm['parts']][:6]}...", file=sys.stderr)
    eps = episodes(t, a.part)
    for e in eps: print(json.dumps(e))
    v = [e["ldlj"] for e in eps if e["ldlj"] == e["ldlj"]]
    print(json.dumps({"ldlj_mean": round(float(np.mean(v)), 2) if v else None, "episodes": len(eps)}))
