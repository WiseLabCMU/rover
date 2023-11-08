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
    python collect.py
    ```

3. In mmWave studio, select `radar/scripts/manual_start.lua` in the bottom dropdown, and click `run`. The data collection script should show status messages as data is collected:
    ```
    [t=1.104s] Flushing 8192 packets.
    [t=1.838s] Flushing 8192 packets.
    ...
    ```
    - Data collection will create around 1GB of data per minute.

4. When finished, select `radar/scripts/manual_stop.lua` in the bottom dropdown, and click `run`.
    - This will create a temporary file at the path specified in `build.py` (default: `rover/radar/tmp.bin`) which you can delete.
    - The data collection setup should raise an error (or time out).

## Data Processing

**Cartographer**:

0. Copy the output Lidar and Radar data to the same folder, and name them `lidar.bag` and `radarpackets.h5` respectively.
1. Run cartographer:
    ```sh
    roslaunch slam offline_cart_3d.launch bag_filenames:=lidar.bag
    ```
    - **NOTE**: you will need to *manually* kill the process with `ctrl+c` after it completes.
2. Generate pose graph:
    ```sh
    roslaunch slam assets_writer_cart_3d.launch \
        bag_filenames:=lidar.bag pose_graph_filename:=lidar.bag.pbstream
    ```
    - This step also creates several output map images (`lidar.bag_xray_{xy, yz, xz}_all.png`, `lidar.bag_probability_grid.png`); verify that these images look about right for the scene you've captured.
3. Output poses:
    ```sh
    rosrun cartographer_ros cartographer_dev_pbstream_trajectories_to_rosbag \
        --input lidar.bag.pbstream --output pose.bag
    rostopic echo -b pose.bag -p trajectory_0 > trajectory.csv
    ```

**Radar Processing**:

0. Only the `radarpackets.h5` and `trajectory.csv` files are needed for radar processing.

1. Process radar data:
    ```sh
    python preprocess.py -p <dataset_folder>
    ```
    - Pass `<dataset_folder>` as the name of the folder containing `radarpackets.h5` and `trajectory.csv` (i.e. the dataset path).
    - You may need to tune `--smooth 1.0`, which indicates the gaussian smoothing width (higher `--smooth` == more smoothing) to be applied to the velocity.
    - You may also need to reduce `--batch` if your GPU does not have enough memory (default: `--batch 1000000`)

2. (Optional) Verify time synchronization:
    ```sh
    python speed_report.py <dataset_folder>
    ```
    - Check that the inferred speed mostly matches the measured/processed speed.
    - Make sure that not too much of the dataset is invalid (due to speeds outside the specified window).
