## dependency
sudo apt-get install ros-indigo-velodyne


## demo 
```
cd didi-data/car/testing
rosbag play ford01.bag -l

rosrun velodyne_pointcloud transform_node _frame_id:=/map

cd ros/
catkin_make

rosrun sensor_fusion image_pointcloud_sync
```

open a new terminal
```
rviz
```


