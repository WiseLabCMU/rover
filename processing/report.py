"""Plot velocity report for sanity checking."""

from argparse import ArgumentParser
import os
import h5py
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt


_desc = """Plot velocity report for sanity checking."""


def _parse(p):
    p.add_argument("-p", "--path", help="Dataset path.")
    p.add_argument("-n", "--rows", help="Number of rows to plot.")
    return p

def _main(args):

    mpl.rcParams.update({'font.size': 8})

    f = h5py.File(os.path.join(args.path, "data.h5"))
    t = np.array(f['t'])

    speed_radar = np.array(f['speed'])
    speed_raw = np.linalg.norm(np.array(f['vel_raw']), axis=1)
    speed_out = np.linalg.norm(np.array(f['vel']), axis=1)
    t = t - t[0]
    chunks = 8

    fig, axs = plt.subplots(chunks, 1, figsize=(8.5, 11))
    for ax in axs:
        ax.plot(t, speed_radar, label='Inferred speed', linewidth=1)
        ax.plot(t, speed_raw, label='Measured speed', linewidth=1)
        ax.plot(t, speed_out, label='Processed speed', linewidth=1)
        ax.plot(
            t, -0.05 + 0.2 * ((speed_out < 0.89) & (speed_out > 0.2)),
            label='Valid?', linewidth=1)
        ax.axhline(0.89, color='black', linewidth=1, linestyle='--')
        ax.axhline(0.2, color='black', linewidth=1, linestyle='--')
        ax.grid()
        ax.set_ylim(0, 1.1)

    for i, ax in enumerate(axs):
        ax.set_xlim(i * t[-1] / chunks, (i + 1) * t[-1] / chunks)

    axs[0].legend(ncols=4)
    fig.tight_layout()
    fig.savefig(os.path.join(args.path, "speed_report.pdf"))


if __name__ == '__main__':
    _main(_parse(ArgumentParser(description=_desc)).parse_args())
