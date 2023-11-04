# Rover
Radar chirp data collection platform based on the TI AWR1843/DCA1000EVM.

See the [setup guide](setup.md) and [manual data collection instructions](manual.md) for more detailed guides.

## Usage

**Linux Computer**, in the `rover/lidar/` directory:

0. Synchronize time with the windows computer:
    ```sh
    sudo ntpdate dart-radar.local
    ```

1. Initialize ROS nodes.
    ```sh
    make init
    ```

2. On each data collection:
    1. Plug in the LIDAR. Wait until you can hear/feel the LIDAR reaching a steady state after spinning up.
    2. Start data collection.
        ```sh
        OUT=out.bag make start
        ```
    3. Stop data collection:
        ```sh
        make stop
        ```
    4. Unplug the LIDAR.

3. Cleanup:
    ```sh
    make deinit
    ```
