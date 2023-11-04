# Rover
Radar chirp data collection platform based on the TI AWR1843/DCA1000EVM.


## Setup Instructions

### Physical Hardware

- **[AWR1843Boost Evaluation Board](https://www.ti.com/tool/AWR1843BOOST)**; uses a 5v 3A power supply.
- **[DCA1000EVM Capture Card](https://www.ti.com/tool/DCA1000EVM)**; powered via the AWR1843.
- **2 micro USB cables** connected to the AWR1843 and to the `RADAR_FTDI` port on the DCA1000EVM.
    - **NOTE**: sketchy USB cables may cause the radar/capture card to fail to be detected.
- **1 Windows computer** (GUI required) for Radar data collection.
- **1 Ubuntu 20.04 (focal) computer** for Lidar/IMU data collection.
- **1 AC Battery Bank** for powering the setup, with an additional power strip if required.

Control System Options:
1. RDP + SSH (Recommended for handheld operation):
    - **1 Windows laptop** for controlling the Windows computer (via Windows RDP).
    - **1 (Wireless) Router** that both computers and the laptop should connect to. Can be replaced with a wired solution if multiple LAN ports are available on each computer (adapters are ok).
    - Make sure to connect both computers to the same network, and assign them known host names (e.g. `dart-lidar` and `dart-radar`).
    - Set static IPs to the radar and lidar computers.
2. Manual Control (Recommended with a cart):
    -  Use a laptop for the Windows and Linux computers, or connect external displays and keyboards to each.

### Radar (Windows) Computer

0. If using RDP to control the data collection system, set up a RDP connection on the Windows laptop.
    1. Open the `Remote Desktop Connection` program (should be a default program on Windows).
    2. Enter the computer name that you've set, i.e. `dart-radar.local`, and your username.
    3. (Optional) Create a shortcut: click `Show Options`, then `Save As...`.

1. If manual time synchronization is required, set up the Windows computer as a NTP server.
    - You can also use the linux computer as a NTP server or an external NTP server (preferred). Only follow these instructions if that is not possible.
    - See full instructions [here](https://techlibrary.hpe.com/docs/otlink-wo/How-to-Configure-a-Local-NTP-Server.html).

2. Install mmWave studio & dependencies. See the [DCA1000EVM Quick Start Guide](https://www.ti.com/tool/DCA1000EVM) for full instructions.
    - Make sure to set a static IP address (`192.168.30.33`) for the network interface used as shown in step 3.
    - Make sure to install the matlab runtime engine noted in step 4.
    - You may need to install the [mmWave SDK](https://www.ti.com/tool/MMWAVE-SDK).
    - Make sure that all 6 COM ports are detected as shown in Step 6 / Figure 4. If the `XDS110 Class` ports are not detected, see the note about installing [EMUPACK](http://processors.wiki.ti.com/index.php/XDS_Emulation_Software_Package).
    - **NOTE**: you may need to disable windows firewall, which might block the ports used by the radar by default.

3. Clone this repository and install dependencies.
    ```sh
    git clone git@github.com:thetianshuhuang/rover
    ```
    - **NOTE**: submodules (i.e. recursive cloning) are not used by the radar data collection system.
    - Install Python >3.8, and `pip install -r radar/requirements.txt`.

4. Configure the lua scripts.
    - Note which COM port is labeled "XDS110 Class Application/User UART". You can verify that you have selected the correct one using mmWave studio:
        1. Open mmWave studio.
        2. Click the `Set` button under `Board Control / Reset Control / Reset`. Wait for the board to finish resetting.
        3. Select the COM port in the `Board Control / RS232 Operations` dropdown.
        4. Click `Connect`. The `RS232 Connectivity Status` field should turn to `Connected`.
    - In the `rover/radar` directory, build lua scripts from templates, replacing `COM4` with the appropriate COM port name.
        ```sh
        python build.py --com COM4
        ```

### LIDAR (Ubuntu 20.04) Computer

0. If using SSH to control the data collection system, make sure to install/enable SSH.
    - A virtual terminal client like `screen`, `tmux`, `zellij` is also helpful. We use `screen` in these instructions (`sudo apt-get -y install screen`).

1. Synchronize time if required (`sudo apt-get install -y ntpdate` if needed):
    ```sh
    sudo ntpdate dart-radar.local
    ```

2. Set `Link-Local` only. In the GUI:
    1. Go to Settings/Network. Click on the ''settings'' button next to the wired connection.
    2. In the IPv4 tab, set `IPv4 Method` to `Link-Local Only`.
    3. In the IPv6 tab, set `IPv6 Method` to `Link-Local Only`.
    4. Disable and re-enable the wired connection.

3. Install [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
    - **NOTE**: make sure you activate the ROS environment on each terminal you use to run subsequent ROS commands:
        ```sh
        source /opt/ros/noetic/setup.bash
        ```

4. Clone this repository. Make sure to clone all subrepositories!
    ```sh
    git clone git@github.com:thetianshuhuang/rover --recursive
    ```
    -  Use `git submodule update --init --recursive` if you forgot to `git clone --recursive`.

5. Install ROS dependencies.
    - Install dependencies for [ouster-ros](https://github.com/ouster-lidar/ouster-ros).
    - Install `python3-catkin-tools`:
        ```sh
        sudo apt-get -y install python3-catkin-tools
        ```
    - Follow manual installation instructions in `lidar/catkin_ws/src/xsens_ros_mti_driver`:
        ```sh
        pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
        ```
6. Build:
    ```sh
    source /opt/ros/noetic/setup.bash
    catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
    - Use `catkin clean` to delete all built files if a rebuild is needed.
    - `--cmake-args -DCMAKE_BUILD_TYPE=Release` is needed for `ouster-ros`.


## Manual Data Collection

### Lidar

**Setup**:

0. **WARNING**: Keep the LIDAR unplugged when not in use to avoid overheating (and potential damage).

1. Set up the terminal (in the `rover/lidar` directory):
    ```sh
    source /opt/ros/noetic/setup.bash
    source catkin_ws/devel/setup.bash
    ```

2. Start `roscore`, `xsens_ros_mti_driver`. In the `lidar` directory:
    ```sh
    screen -S roscore -dm bash -c "roscore"
    screen -S imu -dm bash -c "roslaunch xsens_mti_driver xsens_mti_node.launch"
    ```

**On each data collection**:

3. Plug in the LIDAR, and start the LIDAR in ROS.
    ```sh
    screen -S lidar -dm bash -c "roslaunch data_collection lidar.launch"
    ```

4. Start collecting data.
    ```sh
    rosbag record -O test.bag /imu/data /ouster/imu /ouster/points
    ```
    - Data collection will create around 3.5GB of data per minute.

5. When finished:
    - Use `ctrl+C` (on `rosbag record...`) to kill data collection when finished.
    - Unplug the LIDAR.
    - Kill the `lidar` ROS process with `ctrl+C` or, if using `screen`, send `ctrl+C` to the screen without attaching it with the command
        ```sh
        screen -S lidar -p 0 -X stuff "^C"
        ```

### Radar

**Setup**:

0. Launch mmWave Studio, and wait for the initialization script to finish.

1. Select the `radar/manual_init.lua` script in the bottom dropdown, and click `run` on the left.
    - This configures the radar and capture card. Due to software jankiness in mmWave studio, both must be re-flashed each time mmWave studio is restarted.

**On each data collection**:

2. Start the data collection script. This "steals" the radar socket from mmWave studio.
    ```sh
    python todo.py todo todo
    ```

3. In mmWave studio, select `radar/manual_start.lua` in the bottom dropdown, and click `run`. The data collection script should show status messages as data is collected:
    ```
    todo todo todo
    ```

4. When finished, select `radar/manual_stop.lua` in the bottom dropdown, and click `run`.
    - This will create a temporary file at `todo` which you can delete.
    - The data collection setup should raise an error (or time out).
