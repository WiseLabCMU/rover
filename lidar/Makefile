#     ______  _____  _    _ _______  ______
#    |_____/ |     |  \  /  |______ |_____/
#    |    \_ |_____|   \/   |______ |    \_
#    Ouster LIDAR/Xsens IMU data collection
#

# -- ROS node setup -----------------------------------------------------------

ROSINIT=source /opt/ros/noetic/setup.bash; source catkin_ws/devel/setup.bash
ROSCORE="$(ROSINIT); roscore"
ROSIMU="$(ROSINIT); roslaunch xsens_mti_driver xsens_mti_node.launch"
ROSLIDAR="$(ROSINIT); roslaunch data_collection lidar.launch"

.phony: roscore imu lidar
sleep-%:
	sleep $(@:sleep-%=%)

roscore:
	screen -S roscore -dm bash -c $(ROSCORE)

imu:
	screen -S imu -dm bash -c $(ROSIMU)

lidar:
	screen -S lidar -dm bash -c $(ROSLIDAR)


# -- Data collection ----------------------------------------------------------

TOPICS=/imu/data /ouster/imu /ouster/points
OUT?=out.bag

.phony: init start collect stop deinit
init: roscore sleep-0.5 imu

start: lidar sleep-2 collect

collect:
	screen -S collect -dm bash -c "$(ROSINIT); rosbag record -O $(OUT) $(TOPICS)"

stop:
	screen -S collect -p 0 -X stuff "^C"
	screen -S lidar -p 0 -X stuff "^C"

deinit:
	screen -S imu -p 0 -X stuff "^C"
	screen -S roscore -p 0 -X stuff "^C"
