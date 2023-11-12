"""Plot velocity report for sanity checking."""

import os
import h5py
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt


def _parse(p):
    p.add_argument("-p", "--path", help="Dataset path.")
    p.add_argument(
        "-n", "--rows", default=8, type=int, help="Number of rows to plot.")
    return p


def _main(args):

    mpl.rcParams.update({'font.size': 8})

    radar = h5py.File(os.path.join(args.path, "radar.h5"))
    slam = h5py.File(os.path.join(args.path, "trajectory.h5"))

    t_radar = np.array(radar['t'])[slam['mask']]
    t_radar = t_radar - t_radar[0]
    t_slam = slam['t'] - slam['t'][0]
    speed_radar = np.array(radar['speed'])[slam['mask']]
    speed_slam = np.linalg.norm(np.array(slam['vel']), axis=1)

    valid = np.array(slam['valid'])
    n_valid = np.sum(valid)
    _valid = valid * 0.1
    _valid[valid == 0] = np.nan

    fig, axs = plt.subplots(args.rows, 1, figsize=(8.5, 11))
    for ax in axs:
        ax.plot(t_slam, speed_slam, label='Measured speed', linewidth=1)
        ax.plot(t_radar, speed_radar, label='Inferred speed', linewidth=1)
        ax.plot(
            t_slam, _valid,linewidth=1,
            label='Valid ({}/{})'.format(n_valid, _valid.shape[0]))
        ax.axhline(0.89, color='black', linewidth=1, linestyle='--')
        ax.axhline(0.2, color='black', linewidth=1, linestyle='--')
        ax.grid()
        ax.set_ylim(0, 1.1)

    for i, ax in enumerate(axs):
        ax.set_xlim(
            i * t_slam[-1] / args.rows, (i + 1) * t_slam[-1] / args.rows)

    axs[0].legend(ncols=4)
    fig.tight_layout()
    fig.savefig(os.path.join(
        args.path, "{}_speed_report.pdf".format(os.path.basename(args.path))))
