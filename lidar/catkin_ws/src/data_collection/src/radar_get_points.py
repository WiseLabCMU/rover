#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np
import sys

file_num = sys.argv[1]

f = open('./radar_pc/' + file_num + '.txt', 'w+')

np.set_printoptions(linewidth=40000)

final_pcl = []

def callback(pcl_msg):
    points = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcl_msg, remove_nans=True)
    
    np.savetxt(f, points)
    f.write('\n')

rospy.init_node('data_collection_node', anonymous=True, disable_signals=True)
rospy.Subscriber("/ti_mmwave/radar_scan_pcl", PointCloud2, callback)

start_time_utc = str(rospy.get_rostime())
print('start time: ' + start_time_utc)

rate = rospy.Rate(20)

while True:
    try:
        rospy.spin()
    except KeyboardInterrupt:
        end_time_utc = str(rospy.get_rostime())
        print('end time: ' + end_time_utc)

        np.save('./radar_timestamp/' + file_num + '_radar_pcl_timestamp', np.array([start_time_utc, end_time_utc]))
        exit()
