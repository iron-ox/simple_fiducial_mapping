**NOTE: This package is not maintained; it is meant as a example code only.**

This package provides mapping and localization using apriltag fiducials. It uses detected fiducial positions in images as
input, and solves for a set of camera poses and fiducial positions that minimize the fiducial observation error.

## Building

We use a forked version of apriltags2_ros which we need from source. To get that, put the `.rosinstall` file into your
source directory and run `wstool up`. Then you can build the workspace with catkin_make.

## Creating a Map

First record a bagfile. To do this, on the robot run

```
roslaunch simple_fiducial_mapping record_mapping_bag.launch camera:=<namespace of camera>
```

drive the robot around so that it sees all of the tags from multiple different angles. The resulting bag file
contains fiducial detections, tf transforms, and downsampled images. The images are only needed for debugging. The bag
file is named according the current time and saved in `~/.ros`.

Start a roscore and then solve for the map

```
rosrun simple_fiducial_mapping map_from_bag _bag_file:=<absolute path to bag file>
```

The map will be saved as `output_map.yaml`, and visualization markers are published so that you can view the map in
rviz.

Currently, the only two optimization steps are:
- "pose_check" - quick optimization step to estimate how far we've moved for a new pose.
- "graph_update" - if we've moved far enough, we keep the new pose in the graph and do further optimization.

## Localization

The `localization_node` uses fiducial positions from a map to solve for the robot's current position. It works in much
the same way as mapping, but holds the fiducial positions constant and only solves for the robot's pose.
