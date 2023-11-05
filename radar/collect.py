"""Radar data collection.

::
    ______  _____  _    _ _______  ______
   |_____/ |     |  \  /  |______ |_____/
   |    \_ |_____|   \/   |______ |    \_
   AWR1843Boost/DCA1000EVM Data Collection 

Uses the following procedure:
0. We assume mmWave Studio is already running, and the radar + capture card are
   configured and connected.
1. Connect to the data and control sockets. This "steals" the sockets from
   mmWave Studio.
2. Send a "start" message to the server (`templates/server.lua`) running inside
   mmWave Studio's embedded lua interpreter. The lua server then issues a
   `StartFrame()` command to the DCA1000EVM capture card.
3. Collect data until receiving `ctrl+C` (or timing out); then send a "stop"
   message to the server, which issues `StopFrame()` to the capture card.
"""

import argparse
import time
import numpy as np
import json
import socket
import struct
import tables as tb
from datetime import datetime


PACKET_BUFSIZE = 8192
MAX_PACKET_SIZE = 4096


class Packet(tb.IsDescription):
    """Raw radar packet data type."""
    t = tb.Float64Col()
    packet_data = tb.UInt16Col(shape=(728,))
    packet_num = tb.UInt32Col()
    byte_count = tb.UInt64Col()


class DataCollector:
    """Data collection h5 table management."""

    def __init__(self, h5file):
        scan_group = h5file.create_group('/', 'scan', 'Scan information')
        self.packets = h5file.create_table(
            scan_group, 'packet', Packet, 'Packet data')

        self.total_packets = 0
        self.chunk_packets = 0

        self.start_time = time.time()
    
    def write_packet(self, packet_num, byte_count, packet_data):
        """Write packet to h5 file."""
        self.packets.row['t'] = time.time()
        self.packets.row['packet_data'] = packet_data
        self.packets.row['packet_num'] = packet_num
        self.packets.row['byte_count'] = byte_count
        self.packets.row.append()
        self.chunk_packets += 1

    def flush(self):
        """Flush packets to file."""
        print('[t={:.3f}s] Flushing {} packets.'.format(
            time.time() - self.start_time, self.chunk_packets))
        self.packets.flush()
        self.total_packets += self.chunk_packets
        self.chunk_packets = 0


def _read_data_packet(data_socket):
    """Helper function to read in a single ADC packet via UDP.

    The format is described in the [DCA1000EVM user guide](
        https://www.ti.com/tool/DCA1000EVM#tech-docs)::

        | packet_num (u4) | byte_count (u6) | data ... |

    The packet_num and byte_count appear to be in little-endian order.

    Returns
    -------
    packet_num: current packet number
    byte_count: byte count of data that has already been read
    data: raw ADC data in current packet
    """
    data = data_socket.recv(MAX_PACKET_SIZE)
    # Little-endian, no padding
    packet_num, byte_count = struct.unpack('<LQ', data[:10] + b'\x00\x00')
    packet_data = np.frombuffer(data[10:], dtype=np.uint16)
    return packet_num, byte_count, packet_data


def radarcollect(args, cfg):
    # Need to open the config socket, even if we don't use it, in order to
    # "steal" it from mmwave studo; otherwise, this script will crash.
    _config_socket = socket.socket(
        socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    _config_socket.bind((cfg["static_ip"], cfg["config_port"]))

    data_socket = socket.socket(
        socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    data_socket.bind((cfg["static_ip"], cfg["data_port"]))
    data_socket.settimeout(args.timeout)

    # Instruct the mmWave Studio Lua server to instruct the radar to start
    # collecting data.
    with open(cfg["msgfile"], 'w') as f:
        f.write("start")

    with tb.open_file(args.output, mode='w', title='Packet file') as h5file:
        dataset = DataCollector(h5file)
        try:
            while True:
                dataset.write_packet(*_read_data_packet(data_socket))
                if dataset.chunk_packets >= PACKET_BUFSIZE:
                    dataset.flush()
        except Exception as e:
            dataset.flush()
            print("Radar data collection failed. Was the radar shut down?")
            print(e)

    print(f'Total capture time: {time.time() - dataset.start_time}s\n')
    print("Total packets captured ", dataset.total_packets)
    data_socket.close()
    _config_socket.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description="Radar data collection.")
    p.add_argument(
        '--timeout', '-t', type=float, default=30,
        help='Socket timeout in seconds (eg. 30)')
    p.add_argument(
        '--output', '-o', default=None, help="Output path. If blank, "
        "creates a file with the current datetime (.h5).")
    args = p.parse_args()

    with open("config.json") as f:
        cfg = json.load(f)

    if args.output is None:
        args.output = datetime.now().strftime("%Y.%m.%d-%H.%M.%S") + ".h5"

    try:
        radarcollect(args, cfg)
    except KeyboardInterrupt:
        with open(cfg["msgfile"], 'w') as f:
            f.write("stop")
