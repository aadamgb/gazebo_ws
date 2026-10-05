#!/usr/bin/env python3
# converts a scripted policy (.pt) into a flat binary the RLGoto plugin reads without libtorch
import struct, sys, torch

m = torch.jit.load(sys.argv[1], map_location="cpu")
tensors = [(n, p.detach().float().contiguous()) for n, p in m.named_parameters()]
if any(n.startswith("encoder.") for n, _ in tensors):
    tensors.append(("params_ref", m.params_ref.float().contiguous()))
    tensors.append(("latent_tanh", torch.tensor([float(m.latent_tanh)])))

with open(sys.argv[2], "wb") as f:
    f.write(b"RLP1" + struct.pack("<I", len(tensors)))
    for name, t in tensors:
        f.write(struct.pack("<I", len(name)) + name.encode())
        f.write(struct.pack("<I", t.dim()) + struct.pack(f"<{t.dim()}I", *t.shape))
        f.write(t.numpy().astype("<f4").tobytes())
