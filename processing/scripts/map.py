"""Create ground truth occupancy grid from point cloud (.ply)."""

import os
import math
from tqdm import tqdm

from plyfile import PlyData
import h5py
import numpy as np


def _parse(p):
    p.add_argument("-p", "--path", help="Dataset (directory) path.")
    p.add_argument(
        "-b", "--batch", default=16 * 1024 * 1024, help="Batch size.")
    p.add_argument(
        "-l", "--lower", nargs='+', type=float, default=None,
        help="Lower coordinate in x y z form.")
    p.add_argument(
        "-u", "--upper", nargs='+', type=float, default=None,
        help="Upper coordinate in x y z form.")
    p.add_argument(
        "--padding", type=float, nargs='+', default=[4.0, 4.0, 2.0],
        help="Region padding relative to trajectory min/max.")
    p.add_argument(
        "--resolution", default=50.0, type=float,
        help="Grid resolution in grid cells per meter.")
    return p


def _set_bounds(args):
    if args.lower is None or args.upper is None:
        args.padding = np.array((args.padding * 3)[:3])
        x = np.array(h5py.File(
            os.path.join(args.path, "trajectory.h5"))["pos"])
        args.lower = np.min(x, axis=0) - args.padding
        args.upper = np.max(x, axis=0) + args.padding
    else:
        assert len(args.lower) == 3
        assert len(args.upper) == 3
        args.lower = np.array(args.lower)
        args.upper = np.array(args.upper)


def _main(args):
    _set_bounds(args)

    data = PlyData.read(os.path.join(args.path, "lidar.bag_points.ply"))
    x = data['vertex']['x']
    y = data['vertex']['y']
    z = data['vertex']['z']
    size = [
        math.ceil((u - lw) * args.resolution)
        for lw, u in zip(args.lower, args.upper)]
    grid = np.zeros(size, dtype=bool)

    for _ in tqdm(range(math.ceil(x.shape[0] / args.batch))):
        ix = ((x[:args.batch] - args.lower[0]) * args.resolution).astype(int)
        iy = ((y[:args.batch] - args.lower[1]) * args.resolution).astype(int)
        iz = ((z[:args.batch] - args.lower[2]) * args.resolution).astype(int)

        mask = (
            (ix > 0) & (ix < size[0])
            & (iy > 0) & (iy < size[1])
            & (iz > 0) & (iz < size[2]))

        grid[ix[mask], iy[mask], iz[mask]] = True
        x = x[args.batch:]
        y = y[args.batch:]
        z = z[args.batch:]

    np.savez(
        os.path.join(args.path, "map.npz"), grid=grid,
        lower=np.array(args.lower),
        upper=np.array(args.upper))
