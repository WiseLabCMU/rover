from os import times
from time import time
import numpy as np
import dateutil.parser as dp
import sys
import pickle
from datetime import datetime as dt

num_files = sys.argv[1]

for ii in range(1, int(num_files)+1):
# for ii in range(35, 36):
    print(ii, 'processing ')
    # load data
    imu_data = np.loadtxt('./imu/parsed_imu_' + str(ii) + '.txt')
    mocap_data = np.loadtxt('./mocap/processed_6dof_mocap_' + str(ii) + '.txt', delimiter=',')

    # find timestamp for sensors
    imu_timestamp = imu_data[:,6]
    mocap_timestamp = mocap_data[:,6]

    # find timestamp for radar
    timestamp = np.load('./radar/' + str(ii) + '_radar_pcl_timestamp.npy')
    radar_start_time = int(timestamp[0]) / 1000
    radar_end_time = int(timestamp[1]) / 1000

    sensor_start_time = np.array([int(radar_start_time), int(imu_timestamp[0]), int(mocap_timestamp[0])])
    sensor_end_time = np.array([int(radar_end_time), int(imu_timestamp[imu_data.shape[0] - 1]), int(mocap_timestamp[mocap_data.shape[0] - 1])])

    # check timestamp digits
    for i in range(0, len(sensor_start_time)):
        if len(str(sensor_start_time[i])) == 15:
            sensor_start_time[i] *= 10

    for i in range(0, len(sensor_end_time)):
        if len(str(sensor_end_time[i])) == 15:
            sensor_end_time[i] *= 10

    # print(sensor_start_time)
    # print(sensor_end_time)

    # find start/end time
    global_start_time = np.max(sensor_start_time)
    global_end_time = np.min(sensor_end_time)

    np.save('./timestamp_files/' + str(ii) + '_global_start_end_timestamp', np.array([global_start_time, global_end_time, int(radar_start_time), int(radar_end_time)]))
