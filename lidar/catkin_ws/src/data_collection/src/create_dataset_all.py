import sys
import os

num_files = sys.argv[1]

print(num_files)
print(int(num_files))

os.system('python ./imu/parse_imu_all.py' + ' ' + num_files)
# os.system('python ./radar/read_pkl_all.py' + ' ' + num_files)
os.system('python timestamp_check.py' + ' ' + num_files)
os.system('python read_pcl_all.py' + ' ' + num_files)

print('all files done')

for index in range(1, int(num_files)+1):
    print('processing ' + str(index))
    os.system('python dataset_create.py ' + str(index))
    print(' ')
