#!/bin/bash

rosbag record -O $1.bag /imu/data /os_cloud_node/imu /os_cloud_node/points
