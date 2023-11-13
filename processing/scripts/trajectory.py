"""Process trajectory created by Cartographer (`trajectory.h5`)."""

import h5py
import os
import json
import numpy as np
from scipy.ndimage import binary_dilation

from process import Trajectory, AWR1843Boost


def _parse(p):
    p.add_argument(
        "-p", "--path",
        help="Dataset path; should contain a `trajectory.csv` file.")
    p.add_argument(
        "-v", "--overwrite", help="Overwrite existing data file.",
        default=False, action='store_true')
    p.add_argument(
        "-s", "--smoothing", default=-1.0, type=float,
        help="Gaussian filter to apply to raw data points.")
    p.add_argument(
        "--min_speed", help="Minimum speed threshold (m/s).",
        default=0.2, type=float)
    p.add_argument(
        "--max_accel", help="Maximum allowed acceleration (m/s^2).",
        default=2.0, type=float)
    p.add_argument(
        "--accel_excl", help="Exclusion width for acceleration violations.",
        default=15, type=int)
    p.add_argument(
        "--speed_excl", help="Exclusion width for speed violations.",
        default=5, type=int)
    return p


def _main(args):

    traj = Trajectory.from_csv(args.path, smooth=args.smoothing)

    radar = AWR1843Boost()
    t_radar = np.array(h5py.File(os.path.join(args.path, "radar.h5"))['t'])

    mask = traj.valid_mask(t_radar, window=radar.frame_time)
    poses = traj.interpolate(t_radar[mask], window=radar.frame_time)

    speed = np.linalg.norm(poses['vel'], axis=1)
    accel = np.concatenate(
        [np.zeros(1), np.diff(speed) / np.diff(t_radar[mask])])

    # Seems like scipy's type hints for binary_dilation are incorrect.
    valid = (
        ~binary_dilation(  # type: ignore
            accel > args.max_accel,
            np.ones(args.accel_excl, dtype=bool))
        & ~binary_dilation(  # type: ignore
            (speed > radar.dmax / 2) | (speed < args.min_speed),
            np.ones(args.speed_excl, dtype=bool)))

    poses['t'] = t_radar[mask]
    poses['mask'] = mask
    poses['valid'] = valid

    outfile = h5py.File(os.path.join(args.path, "trajectory.h5"), 'w')
    for k, v in poses.items():
        outfile.create_dataset(k, data=v)

    with open(os.path.join(args.path, "metadata.json"), 'w') as f:
        keys = [
            "smoothing", "min_speed", "max_accel", "accel_excl", "speed_excl"]
        meta = {k: getattr(args, k) for k in keys}
        json.dump({
            "n_frames": valid.shape[0],
            "total_time": poses['t'][-1] - poses['t'][0], **meta
        }, f, indent=4)
