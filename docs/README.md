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

1. Run the python 3 predict service
```
$ python ./run_model.py -m ped 
```

2. create the ros workspace in the ros dir
```
$ catkin_make
```
3. source ros env
```
$ source devel/setup.bash
```

the tag is the tag for taining weights directory name, which tag is your training tag.

4. run all from ros launch
```
roslaunch ped.launch  bag:=ped_test rate:=0.3 bag_DIR:=/media/stu/hdd1_1t/competition_data/didi_dataset/round2/test_ped
```

if you run the launche file
it will run automaticly
- detector node
- tracker node
- projection node
- RViz tooks
- and rosbag play the bag which you need

and the tracklet xml will create in the current directory

in the visible tools
- the yellow box is the result predicted by network,
- the green box is the result by kalman filter in tracker ros node

the launch file is
and <<bag>> and <<rate>> is defined in the launch xml
``` xml
<launch>
    <!--change it to your bag directory-->
    <arg name="bag_DIR" default="/ext/Data/Round_2/release/" />
    <!--change it to the mame which bag you need to run-->
    <arg name="bag" default="car/testing/ford01" />
    <arg name="rate" default="0.2" />

    <node name="detector" pkg="detector" type="pipeline.py"/>
    <node name="tracker" pkg="tracker" type="tracker.py"/>

    <node name="projection" pkg="projection" type="visualize_result.py" args="$(find projection)/scripts/ost_new.yaml"/>
    <node name="velodyne" pkg="velodyne_pointcloud" type="cloud_node" args="">
        <param name="calibration" value="$(find velodyne_pointcloud)/params/32db.yaml" />
    </node>

    <node name="rosbag" pkg="rosbag" type="play" args="-r $(arg rate) -l --clock $(arg bag_DIR)/$(arg bag).bag"/>
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find projection)/launch/config-result.rviz"  />
</launch>

```