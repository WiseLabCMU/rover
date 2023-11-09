#     ______  _____  _    _ _______  ______
#    |_____/ |     |  \  /  |______ |_____/
#    |    \_ |_____|   \/   |______ |    \_
#    Data Processing Pipeline (Cartographer)
#

DIR?=$(shell pwd)

# -- Cartographer -------------------------------------------------------------

LIDAR?=lidar.bag
PBSTREAM=$(DIR)/$(LIDAR).pbstream
POINTS=$(DIR)/$(LIDAR)_points.ply
POSE=$(DIR)/pose.bag
TRAJ=$(DIR)/trajectory.csv

.precious: $(PBSTREAM)
.phony: lidar
lidar: $(POINTS) $(TRAJ)

$(PBSTREAM):
	-roslaunch slam offline_cart_3d.launch bag_filenames:=$(DIR)/$(LIDAR)

$(POINTS): $(PBSTREAM)
	roslaunch slam assets_writer_cart_3d.launch \
		bag_filenames:=$(DIR)/$(LIDAR) pose_graph_filename:=$(PBSTREAM)

$(POSE): $(PBSTREAM)
	rosrun cartographer_ros cartographer_dev_pbstream_trajectories_to_rosbag \
		--input $(PBSTREAM) --output $(POSE)

$(TRAJ): $(POSE)
	rostopic echo -b $(POSE) -p trajectory_0 > $(TRAJ)

# -- Python -------------------------------------------------------------------

FLAGS?=
RADAR=$(DIR)/radar.h5
TRAJ_H5=$(DIR)/trajectory.h5
REPORT=$(DIR)/report.pdf

.phony: process
process: $(RADAR) $(TRAJ_H5) $(REPORT)

$(RADAR):
	python process.py fft -p $(DIR)

$(TRAJ_H5):
	python process.py trajectory -p $(DIR) $(FLAGS)

$(REPORT): $(RADAR) $(TRAJ_H5)
	python process.py report -p $(DIR)
