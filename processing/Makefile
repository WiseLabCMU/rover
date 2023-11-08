# Cartographer processing pipeline.

LIDAR?=lidar.bag
DIR?=$(shell pwd)
PBSTREAM=$(DIR)/$(LIDAR).pbstream
POINTS=$(DIR)/$(LIDAR)_points.ply
POSE=$(DIR)/pose.bag
TRAJ=$(DIR)/trajectory.csv

.phony: all
all: $(POINTS) $(TRAJ)

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
