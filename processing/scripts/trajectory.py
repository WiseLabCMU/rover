"""Process trajectory created by Cartographer.

Takes a `trajectory.csv` file and already-processed `radar.h5` file, and
outputs a `trajectory.h5` processed file.
"""

import h5py
import os
import numpy as np
from scipy.ndimage import binary_dilation

from process import Trajectory, AWR1843Boost


def _parse(p):
    p.add_argument("-p", "--path", help="Dataset path.")
    p.add_argument(
        "-v", "--overwrite", help="Overwrite existing data file.",
        default=False, action='store_true')
    p.add_argument(
        "-s", "--smooth", help="Gaussian filter to apply to raw data points.",
        default=-1.0, type=float)
    p.add_argument(
        "--min_speed", help="Minimum speed threshold (m/s).",
        default=0.2, type=float)
    p.add_argument(
        "--max_accel", help="Maximum allowed acceleration (m/s^2).",
        default=0.5, type=float)
    p.add_argument(
        "--accel_excl", help="Exclusion width for acceleration violations.",
        default=15, type=int)
    p.add_argument(
        "--speed_excl", help="Exclusion width for speed violations.",
        default=5, type=int)
    return p


def _main(args):

    traj = Trajectory.from_csv(args.path, smooth=args.smooth)

    radar = AWR1843Boost()
    t_radar = np.array(h5py.File(os.path.join(args.path, "radar.h5"))['t'])

    mask = traj.valid_mask(t_radar, window=radar.frame_time)
    poses = traj.interpolate(t_radar[mask], window=radar.frame_time)

    speed = np.linalg.norm(poses['vel'], axis=1)
    accel = np.concatenate(
        [np.zeros(1), np.diff(speed) / np.diff(t_radar[mask])])
    valid = (
        ~binary_dilation(
            accel > args.max_accel,
            np.ones(args.accel_excl, dtype=bool))
        & ~binary_dilation(
            (speed > radar.dmax / 2) & (speed < args.min_speed),
            np.ones(args.speed_excl, dtype=bool)))

    poses['t'] = t_radar[mask]
    poses['mask'] = mask
    poses['valid'] = valid

    outfile = h5py.File(os.path.join(args.path, "trajectory.h5"), 'w')
    for k, v in poses.items():
        outfile.create_dataset(k, data=v)
