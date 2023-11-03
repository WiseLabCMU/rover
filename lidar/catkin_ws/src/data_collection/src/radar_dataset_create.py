import sys
import numpy as np

file_num = sys.argv[1]

timestamp = np.load('./timestamp_files/' + file_num + '_global_start_end_timestamp' + '.npy')
start_time = timestamp[0]
end_time = timestamp[1]

radar_start_time = timestamp[2]
radar_end_time = timestamp[3]

# 20Hz
radar_timestamp = np.arange(start=radar_start_time, stop=radar_end_time, step=50000)
radar_left_index = np.argmax(radar_timestamp >= (start_time + 1))
radar_right_index = np.argmin(radar_timestamp <= (end_time - 1))

radar_data = np.load('radar_pcl_' + file_num)

final_depth = radar_data[radar_left_index : radar_right_index]

np.save('radar_dataset_' + file_num, final_depth)