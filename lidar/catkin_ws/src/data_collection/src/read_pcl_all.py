import sys
import os

num_files = sys.argv[1]

for index in range(1, int(num_files)+1):
    os.system('python read_pcl.py ' + str(index))
    print(' ')
