"""The watch-it-learn clip: eval clips of successive checkpoints, cut together, each with its training step,
its held-out hit rate, and a live counter of hits over attempts in the clip.

    uv run python learn_clip.py out.mp4 --segment clip.mp4 take.web.json "label" 0.25 [--segment ...] [--seconds 9]

Each clip is an eval reel (Assets/Showcase/Reels/eval.json): one shot that starts at the take's second episode,
so video time t is take time episodes[1] + t. Attempts are episodes; hits come from the take.
"""
import argparse, json, subprocess, tempfile
from pathlib import Path
import imageio_ffmpeg

FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def esc(s):
    return s.replace("\\", "\\\\").replace(":", "\\:").replace("'", "’")


def segment(clip, web, label, rate, seconds, out):
    j = json.loads(Path(web).read_text())
    t0 = j["episodes"][1] if len(j["episodes"]) > 1 else 0.0
    starts = [e - t0 for e in j["episodes"] if 0 <= e - t0 < seconds]
    hits = [h - t0 for h in j["hits"] if 0 <= h - t0 < seconds]
    # Counter text as piecewise intervals: at time t, attempts = episodes started by t, hits = hits by t.
    events = sorted(set([0.0] + starts + hits + [seconds]))
    draws = []
    for a, b in zip(events, events[1:]):
        n_att = max(1, sum(1 for s in starts if s <= a + 1e-6))
        n_hit = sum(1 for h in hits if h <= a + 1e-6)
        draws.append(f"drawtext=expansion=none:fontfile={FONT}:text='{esc(f'hits {n_hit} / {n_att}')}':x=w-tw-64:y=h-th-56:fontsize=34:fontcolor=0xE9E6E0:"
                     f"enable='between(t,{a:.3f},{b:.3f})'")
    flash = "".join(f",drawbox=x=0:y=0:w=iw:h=ih:color=0xFFC468@0.10:t=fill:enable='between(t,{h:.3f},{h + 0.25:.3f})'" for h in hits)
    vf = (f"trim=0:{seconds},setpts=PTS-STARTPTS,scale=1920:1080,"
          f"drawtext=expansion=none:fontfile={BOLD}:text='{esc(label)}':x=64:y=56:fontsize=40:fontcolor=0xE9E6E0,"
          f"drawtext=expansion=none:fontfile={FONT}:text='{esc(f'held-out hit rate {rate * 100:.0f}%')}':x=64:y=110:fontsize=30:fontcolor=0xFFC468,"
          + ",".join(draws) + flash + f",fade=t=in:st=0:d=0.4,fade=t=out:st={seconds - 0.4}:d=0.4")
    subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-threads", "4", "-i", clip, "-vf", vf, "-an", "-r", "30",
                    "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p", out], check=True)
    return len(hits), len(starts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--segment", nargs=4, action="append", metavar=("CLIP", "WEB_JSON", "LABEL", "HIT_RATE"), required=True)
    ap.add_argument("--seconds", type=float, default=9.0)
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        parts = []
        for i, (clip, web, label, rate) in enumerate(a.segment):
            p = Path(tmp) / f"{i:02d}.mp4"
            h, n = segment(clip, web, label, float(rate), a.seconds, str(p))
            print(f"{label}: {h} hits / {n} attempts in {a.seconds:.0f} s")
            parts.append(p)
        lst = Path(tmp) / "list.txt"
        lst.write_text("".join(f"file '{p}'\n" for p in parts))
        subprocess.run([FFMPEG, "-y", "-loglevel", "error", "-threads", "4", "-f", "concat", "-safe", "0", "-i", str(lst),
                        "-c:v", "libx264", "-crf", "18", "-preset", "slow", "-pix_fmt", "yuv420p", "-movflags", "+faststart", a.out], check=True)
    print(a.out)


if __name__ == "__main__":
    main()
