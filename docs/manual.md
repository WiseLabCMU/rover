# Manual Data Collection

These steps assume you have already set up the system according to the [instructions](setup.md).

## Lidar

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

3. Plug in the LIDAR. Wait until you can hear the LIDAR spinning; then, start the LIDAR in ROS.
    ```sh
    screen -S lidar -dm bash -c "roslaunch data_collection lidar.launch"
    ```

4. Start collecting data.
    ```sh
    rosbag record -O out.bag /imu/data /ouster/imu /ouster/points
    ```
    - Data collection will create around 3.5GB of data per minute.

5. When finished:
    - Use `ctrl+C` (on `rosbag record...`) to kill data collection when finished.
    - Unplug the LIDAR.
    - Kill the `lidar` ROS process with `ctrl+C` or, if using `screen`, send `ctrl+C` to the screen without attaching it with the command
        ```sh
        screen -S lidar -p 0 -X stuff "^C"
        ```
    - Inspect the created bag with `rosbag info out.bag`.

## Radar

**Setup**:

0. Launch mmWave Studio, and wait for the initialization script to finish.

1. Select the `radar/scripts/manual_init.lua` script in the bottom dropdown, and click `run` on the left.
    - This configures the radar and capture card. Due to software jankiness in mmWave studio, both must be re-flashed each time mmWave studio is restarted.

**On each data collection**:

2. Start the data collection script. This "steals" the radar socket from mmWave studio.
    ```sh
    python todo.py todo todo
    ```

3. In mmWave studio, select `radar/scripts/manual_start.lua` in the bottom dropdown, and click `run`. The data collection script should show status messages as data is collected:
    ```
    todo todo todo
    ```

4. When finished, select `radar/scripts/manual_stop.lua` in the bottom dropdown, and click `run`.
    - This will create a temporary file at the path specified in `build.py` (default: `rover/radar/tmp.bin`) which you can delete.
    - The data collection setup should raise an error (or time out).
