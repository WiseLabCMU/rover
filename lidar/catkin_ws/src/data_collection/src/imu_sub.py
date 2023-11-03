#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import sys

file_num = sys.argv[1]

imu_file = open("./imu/" + file_num + ".txt", "w+")

def callback(imu_msg):
    timestamp = imu_msg.header.stamp

    linear_x = imu_msg.linear_acceleration.x
    linear_y = imu_msg.linear_acceleration.y
    linear_z = imu_msg.linear_acceleration.z

    angular_x = imu_msg.angular_velocity.x
    angular_y = imu_msg.angular_velocity.y
    angular_z = imu_msg.angular_velocity.z

    orientation_x = imu_msg.orientation.x
    orientation_y = imu_msg.orientation.y
    orientation_z = imu_msg.orientation.z
    orientation_w = imu_msg.orientation.w

    data = str(linear_x) + " " + str(linear_y) + " " + str(linear_z) + " " \
             + str(angular_x) + " " + str(angular_y) + " " + str(angular_z) \
                 + " " + str(orientation_x) + " " + str(orientation_y) \
                 + " " + str(orientation_z) + " " + str(orientation_w) + " " + str(timestamp) + "\n"
    
    imu_file.write(data)

rospy.init_node('imu_sub_node', anonymous=True)
rospy.Subscriber("/imu/data", Imu, callback)

rate = rospy.Rate(20)
rospy.spin()

