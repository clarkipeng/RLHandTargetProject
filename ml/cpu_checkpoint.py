"""Rewrite a training checkpoint with CPU tensors, so a CPU-only trainer can resume a run that last trained on the GPU.
    uv run --project gpu python cpu_checkpoint.py results/<run>/Throw/checkpoint.pt [more .pt files]"""
import sys, torch

def cpu(x):
    if torch.is_tensor(x): return x.cpu()
    if isinstance(x, dict): return type(x)((k, cpu(v)) for k, v in x.items())
    if isinstance(x, (list, tuple)): return type(x)(cpu(v) for v in x)
    return x

for path in sys.argv[1:]:
    state = torch.load(path, map_location="cpu")
    torch.save(cpu(state), path)
    print("cpu tensors:", path)
