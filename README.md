# Rover
Radar chirp data collection platform based on the TI AWR1843/DCA1000EVM.

![Data collection system.](docs/equipment.svg)

This repository contains three components, each of which should run on a different machine:
- `lidar`: LIDAR/IMU data collection infrastructure for an Ouster LIDAR + Xsens IMU.
- `radar`: Radar data collection infrastructure for the TI AWR1843Boost + DCA1000EVM.
- `processing`: Radar/LIDAR/IMU data processing to create the final dataset.

## Physical Hardware

- **1 Windows computer** (GUI required) for Radar data collection.
- **1 Ubuntu 20.04 (focal) computer** for Lidar/IMU data collection.
- **[AWR1843Boost Evaluation Board](https://www.ti.com/tool/AWR1843BOOST)**
    - **1 micro USB cable** connecting the AWR1843's USB port to the windows computer.
    - **5v 3A power supply**
    - **NOTE**: sketchy USB cables may cause the radar/capture card to fail to be detected.
- **[DCA1000EVM Capture Card](https://www.ti.com/tool/DCA1000EVM)**
    - Powered via the AWR1843Boost.
    - **1 micro USB cable** connecting the `RADAR_FTDI` port to the windows computer.
- **[Ouster OS0-64 LIDAR](https://ouster.com/products/hardware/os0-lidar-sensor)**
    - **1 Cat5 ethernet cable** connecting the LIDAR interface box to the linux computer.
- **[Xsens MTi-3 AHRS Development Kit](https://shop.movella.com/us/product-lines/sensor-modules/products/mti-3-ahrs-development-kit)**
    - **1 micro USB cable** connecting the IMU's USB port to the linux computer.
- **1 AC Battery Bank** for powering the setup, with an additional power strip if required.

Control System Options:
1. RDP + SSH (Recommended for handheld operation):
    - **1 Windows laptop** for controlling the Windows computer (via Windows RDP).
    - **1 (Wireless) Router** that both computers and the laptop should connect to. Can be replaced with a wired solution if multiple LAN ports are available on each computer (adapters are ok).
    - Make sure to connect both computers to the same network, and assign them known host names (e.g. `dart-lidar` and `dart-radar`).
    - Set static IPs to the radar and lidar computers.
2. Manual Control (Recommended with a cart):
    - **Use a laptop for the Windows and Linux computers**, or connect external displays and keyboards to each.

## Setup

See the [setup guide](docs/setup.md).

## Data Collection Guidelines

1. **Keep a reasonable speed**. DART will reject data when the speed exceeds 0.89 m/s (due to doppler "wrapping") or is less than 0.2 m/s (due to overly wide doppler bins). High speeds may lead to inconsistent velocity estimates by cartographer, so we suggest targeting **0.5m/s**.
2. **Move smoothly**, again to assist cartographer in velocity estimation.
3. **Keep the radar above your head when moving through tight spaces**. The LIDAR has a minimum distance of 0.3m, and can lose tracking in tight spaces if you obscure half of its viewing range.
4. **Scan high and low**. The actual FOV of the radar is relatively narrow, so you should explicitly scan low and high points.
5. **Emphasize occlusion**. Basically any radar processing algorithm will work in a featureless metal box. Try to capture complex geometry and occlusion patterns (e.g. going behind walls, furniture, etc).

## Usage

Note that these steps should be performed simultaneously on the Linux and Windows computer. In particular, `make start` and `python collect.py` should be performed right before the actual data collection step to avoid excess file size.

For a detailed step-by-step breakdown which bypasses any high-level automation for troubleshooting/development, see the [manual data collection instructions](docs/manual.md).

**Linux Computer**, in the `rover/lidar/` directory (collects 3.5GB/minute):

- On reboot (~15 seconds):
    1. Synchronize time with the windows computer: `sudo ntpdate dart-radar.local`
    2. Initialize ROS nodes: `make init`
- On each data collection (~30 seconds):
    1. Plug in the LIDAR. Wait until you can hear/feel the LIDAR reaching a steady state after spinning up.
    2. Start data collection: `OUT=lidar.bag make start` (replace `lidar.bag` with the desired output file name).
        - **NOTE**: it may take *up to 30 seconds* for data to start being collected; one way to check is to watch the file size of `lidar.active.bag`, and wait until it starts to increase rapidly.
        - Starting LIDAR data collection **before** starting radar data collection is suggested.
    3. Stop data collection: `make stop`. A large `lidar.bag` file (several GBs) should be created.
    4. Unplug the LIDAR.
- Cleanup: `make deinit`

**Windows Computer**, in the `rover/radar` directory (collects 1GB/minute):

- On reboot (~2 minutes):
    1. Power on the Radar, and make sure the `XDS110 Class Application/User UART` COM port matches what you have in `config.json`.
    2. Launch mmWave studio, and wait for all initialization commands to complete: `python init.py`
- On each data collection (~5 seconds):
    1. Run `python collect.py`. A `radarpackets.h5` file should be created.
    2. Press `ctrl+C` *once* on `python collect.py` when finished. **Do not close mmWave Studio**, or you will need to restart the whole procedure and reflash the radar and capture card.
- Cleanup: close mmWave studio. The radar can stay powered on.

**Data Processing**, on a separate computer with `cartographer-ros` installed:

1. Copy the collected lidar and radar data to a folder; name the lidar data `lidar.bag`, and the radar data `radarpackets.h5`.

2. Run cartographer (~30 minutes):
    ```sh
    DIR=<dataset_directory> make lidar
    ```
    - Replace `<dataset_directory>` with the folder containing `lidar.bag`; all output files are also placed in this folder.
    - When running the makefile, the first step (`roslaunch slam offline_cart_3d.launch ...`) will wait indefinitely after it finishes if rviz is enabled. If this happens, close the rviz window, and the script will continue. **DO NOT** use `ctrl+c`; this will cancel the makefile as well.
    - Instead of passing `DIR=...`, you can alternatively copy this makefile to the `<dataset_directory>` and simply `make`.
    - This should create a number of files, including `trajectory.csv` and `lidar.bag_points.ply`.

3. Run radar processing & dataset packaging:
    ```sh
    DIR=<dataset_directory> make process
    ```
    - Replace `<dataset_directory>` with the folder containing `trajectory.csv`, `lidar.bag_points.ply`, and `radarpackets.h5` from above (this step can be performed on a different machine, preferrably GPU-accelerated, as long as the above files are copied over).
    - This should create a number of files, including `radar.h5`, `trajectory.h5`, `map.npz`, and `speed_report.pdf`.
