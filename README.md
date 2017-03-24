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
Click [here](./docs/rosbag.md) for an overview of a ROS bag

# ROS quick tutorial

## Navigating the ROS Filesystem
**Explain what is ROS package and manifests,how to find package(rospack find), change directory to a package location (roscd),and list package directory contents (rosls).**

**Packages**: software organization unit of ROS code. Each package can contain libraries, executables, scripts or other artifacts.

**Manifests (package.xml)**: A manifest is a description of a package. It serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc...

* Find package: 
rospack find [package_name]
```
$ rospack find roscpp
/opt/ros/indigo/share/roscpp
```

* Change directory to a package location:
roscd [locationame[/subdir]]
```
$ roscd roscpp
/opt/ros/indigo/share/roscpp$
```

* List package directory contents
rosls
```
/opt/ros/indigo/share/roscpp$ rosls
cmake  msg  package.xml  rosbuild  srv
```










