"""Convert CFAR point cloud to a reflectance map."""

import numpy as np
import os
import h5py
import math
from tqdm import tqdm


def _parse(p):
    p.add_argument("-p", "--path", help="Dataset (directory) path.")
    p.add_argument(
        "-b", "--batch", default=16, help="Batch size.")
    p.add_argument(
        "--padding", type=float, nargs='+', default=[4.0, 4.0, 2.0],
        help="Region padding relative to trajectory min/max.")
    p.add_argument(
        "--resolution", default=10.0, type=float,
        help="Grid resolution in grid cells per meter.")
    return p


def _set_bounds(args):
    args.padding = np.array((args.padding * 3)[:3])
    x = np.array(h5py.File(
        os.path.join(args.path, "trajectory.h5"))["pos"])
    lower = np.min(x, axis=0) - args.padding
    upper = np.max(x, axis=0) + args.padding
    return lower, upper


def _main(args):
    lower, upper = _set_bounds(args)

    data = h5py.File(os.path.join(args.path, "cfar.h5"))
    pos = np.array(data['pos']).T
    sigma = np.array(data['amplitude'])

    order = np.argsort(sigma)
    sigma = sigma[order]
    pos = pos[order]

    size = [
        math.ceil((u - lw) * args.resolution)
        for lw, u in zip(lower, upper)]
    grid = np.zeros(size, dtype=float)

    for _ in tqdm(range(math.ceil(pos.shape[0] / args.batch))):
        x, y, z = ((pos[:args.batch] - lower) * args.resolution).astype(int).T
        mask = (
            (x > 0) & (x < size[0])
            & (y > 0) & (y < size[1])
            & (z > 0) & (z < size[2]))
        grid[x[mask], y[mask], z[mask]] = np.maximum(
            grid[x[mask], y[mask], z[mask]], sigma[:args.batch][mask])
        
        sigma = sigma[args.batch:]
        pos = pos[args.batch:]

    np.savez(
        os.path.join(args.path, "cfar.npz"), grid=grid,
        lower=lower, upper=upper)
