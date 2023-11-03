import numpy as np
from pcl_to_depth import output_img
import sys

file_num = sys.argv[1]

f = open(file_num + '.txt', 'r')
lines = f.read().splitlines()

local_arr = np.zeros((1,3))
final_depth = np.zeros((1,64,256))

for line in lines:
    if line != '':
        data = np.array(line.split(), dtype=float).reshape(1,3)
        local_arr = np.vstack((local_arr, data))

    else:
        depth_img = np.array(output_img(local_arr[1:,])).reshape(1, 64, 256)
        final_depth = np.vstack((final_depth, depth_img))

        print(final_depth.shape)

        local_arr = np.zeros((1,3))

np.save('radar_pcl_' + file_num, final_depth[1:,])
