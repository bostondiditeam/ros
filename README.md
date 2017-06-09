# ros
Boston Didi Challenge matters pertaining to Robot Operating System (ROS)

# Installation

According to the Submission section for the Rules at https://challenge.udacity.com/:
```
Solution must run in the provided ROS framework (Ubuntu 14.04, ROS Indigo)
```

If you are running a Linux host, please be sure to use Ubuntu 14.04.


For instructions on installing ROS using Docker, please click [here](./docs/installation.md)
*(Note: competition requires ROS Indigo, which is not compatible with Ubuntu 16.04).*

# ROS bag
Click [here](./docs/rosbag.md) for an overview of a ROS bag.

Click [here](./docs/rosbag_exploration.ipynb) to look at the Jupyter notebook to get started with exploring the rosbag data from Udacity.

# ROS quick tutorial
Click [here](./docs/ros_quick_tutorial.md) for a quick tutorial of ROS.

1. create the ros workspace in the dir
```
$ catkin_make
```
2. create two ros package
```
$ cd src
$ catkin_create_pkg detector std_msgs rospy roscpp
$ catkin_create_pkg tracker std_msgs rospy roscpp
```

# ROS Nodes

## [detector]
use this command to run before ros play didi test bag
```
$ rosrun detector pipeline.py

```

## [tracker]

