"""Run Range/Doppler/Azimuth FFT and create radar frames.

Takes a `radarpackets.h5` file creating raw radar packets from `collect.py`,
and creates a `radar.h5` file containing raw radar (range, doppler, azimuth)
images.
"""

import os
import json
import h5py
from tqdm import tqdm
import numpy as np

from process import AWR1843Boost, AWR1843BoostDataset


def _parse(p):
    p.add_argument("-p", "--path", help="Dataset path.")
    p.add_argument(
        "-v", "--overwrite", help="Overwrite existing data file.",
        default=False, action='store_true')
    p.add_argument(
        "-b", "--batch", help="Packet processing batch size.",
        type=int, default=1024 * 1024)
    return p


def _open_files(path: str, overwrite: bool = False):
    """Open input/output files.
    
    If overwrite is not set and `radar.h5` is already present, this will raise
    an error on `h5py.File(... 'w')`.
    """
    packetfile = h5py.File(os.path.join(path, "radarpackets.h5"), "r")
    packets = packetfile["scan"]["packet"]

    if overwrite:
        try:
            os.remove(os.path.join(path, 'radar.h5'))
        except OSError:
            pass
    outfile = h5py.File(os.path.join(path, "radar.h5"), 'w')

    return packets, outfile


def _process_batch(radar: AWR1843Boost, file_batch):
    """Thin passthrough to `AWR1843BoostDataset.process_data`."""
    fields = ["byte_count", "packet_data", "packet_num", "t"]
    packets = {k: file_batch[k] for k in fields}
    dataset = AWR1843BoostDataset.from_packets(packets, radar.chirp_size)
    return dataset.process_data(radar, packets)


def _main(args):
    packets, outfile = _open_files(args.path, overwrite=args.overwrite)

    # Replace this when supporting more radars in the future
    radar = AWR1843Boost()
    with open(os.path.join(args.path, "sensor.json"), 'w') as f:
        json.dump(radar.to_instrinsics(), f, indent=4)

    rs = radar.image_shape
    rda_dataset = outfile.create_dataset(
        "rad", (1, *rs), dtype='f2', chunks=(1, *rs), maxshape=(None, *rs))

    total_size = 0
    speed_data, t_data = [], []

    _batches = int(np.ceil(packets.shape[0] / args.batch))
    for _ in tqdm(range(_batches)):
        try:
            rda, speed, t = _process_batch(radar, packets[:args.batch])

            rda_dataset.resize((total_size, *radar.image_shape))
            rda_dataset[-rda.shape[0]:] = rda
            total_size += rda.shape[0]

            speed_data.append(speed)
            t_data.append(t)
        except Exception as e:
            print(e)

        packets = packets[args.batch:]

    outfile.create_dataset("speed", data=np.concatenate(speed_data))
    outfile.create_dataset("t", data=np.concatenate(t_data))
