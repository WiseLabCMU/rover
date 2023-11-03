import h5py
import numpy as np
import sys
import os
from datetime import datetime as dt
from scipy.io import savemat

file_num = sys.argv[1]

# parse mocap
# os.system('python ./mocap/mocap_data_parser.py' + ' ' + file_num + '.txt')

timestamp = np.load('./timestamp_files/' + file_num + '_global_start_end_timestamp.npy')

# 20 Hz
final_timestamp = np.arange(start=timestamp[0], stop=timestamp[1], step=50000)

print(final_timestamp.shape)

# generate imu data (x.npy)
os.system('python ./imu/trim_imu_data.py' + ' ' + file_num)
# get Opti-Track data (mocap_dataset_x.npy)
os.system('python ./mocap/mocap_dataset_generator.py' + ' ' + file_num + ' ' + str(final_timestamp.shape[0]))
# get radar data

# load data
gt = np.load('./mocap/mocap_dataset_' + file_num + '.npy').reshape(1, final_timestamp.shape[0], 6)
imu = np.load('./imu/imu_dataset_' + file_num + '.npy')
radar = np.load('./radar/radar_dataset_' + file_num + '.npy')
radar = radar.reshape(radar.shape[0], 1, 64, 256, 1)
final_timestamp = final_timestamp.reshape(1, final_timestamp.shape[0])

# delete last row if size does not match
length_array = [gt.shape[1], imu.shape[0], radar.shape[0], final_timestamp.shape[1]]
min_len = min(length_array)
if gt.shape[1] > min_len:
    gt = np.delete(gt, gt.shape[1] - 1, axis=1)
if imu.shape[0] > min_len:
    imu = np.delete(imu, imu.shape[0] - 1, axis=0)
if radar.shape[0] > min_len:
    radar = np.delete(radar, radar.shape[0] - 1, axis=0)
if final_timestamp.shape[1] > min_len:
    final_timestamp = np.delete(final_timestamp, final_timestamp.shape[1] - 1, axis=1)

# check shape
print('timestamp: ' + str(final_timestamp.shape))
print('ground truth: ' + str(gt.shape))
print('imu: ' + str(imu.shape))
print('radar: ' + str(radar.shape))

# create dataset
hf = h5py.File('./h5/data_' + file_num + '.h5', 'w')
hf.create_dataset('timestamp', data=final_timestamp)
# hf.create_dataset('label_data', data=gt)
hf.create_dataset('imu_data', data=imu)
hf.create_dataset('mmwave_middle_data', data=radar)
hf.close()

savemat('./mocap/downsampled'+file_num+'.mat',{'mocap':gt})