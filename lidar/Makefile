#     ______  _____  _    _ _______  ______
#    |_____/ |     |  \  /  |______ |_____/
#    |    \_ |_____|   \/   |______ |    \_
#    Ouster LIDAR/Xsens IMU data collection
#

# -- ROS node setup -----------------------------------------------------------

ROSINIT=source /opt/ros/noetic/setup.bash; source catkin_ws/devel/setup.bash
ROSIMU="roslaunch xsens_mti_driver xsens_mti_node.launch"
ROSLIDAR="roslaunch data_collection lidar.launch"

.phony: roscore imu lidar
roscore:
	$(ROSINIT); screen -S roscore -dm bash -c "roscore"

imu:
	$(ROSINIT); screen -S imu -dm bash -c $(ROSIMU)

lidar:
	$(ROSINIT); screen -S lidar -dm bash -c $(ROSLIDAR)


# -- Data collection ----------------------------------------------------------

TOPICS=/imu/data /ouster/imu /ouster/points
OUT?=out.bag

.phony: init
init: roscore imu

.phony: start
start: lidar
	screen -S collect -dm bash -c "rosbag record -O $(OUT) $(TOPICS)"

.phony: stop
stop:
	screen -S collect -p 0 -X stuff "^C"
	screen -S lidar -p 0 -X stuff "^C"
