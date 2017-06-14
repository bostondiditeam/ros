# Function

[Work-In-Progress] Tracker Node for ROS framework to enable real-time processing of ROS bags in Udacity's Didi Chuxing Challenge.

# Overview

* Tracker node subscribes to `MarkerArray` messages from **detector node**, which contains bounding boxes for a single frame
* Tracker also subscribes to camera `/image_raw` messages for time-stamps
* Hungarian and Kalman updates are called every time we have a new bounding box message
* Hungarian algorithm is responsible for calling Kalman filter updates and separate targets into individual tracks.

# Instructions

* Copy the entire `src` folder to your catkin workspace. For example,
```
cp -r src ~/catkin_ws/
cd ~/catkin_ws/
```

* Note: Make sure `~/catkin_ws/src/tracker/scripts/tracker.py` is executable (use  `chmod +x`).

* Build the catkin package
```
catkin_make
source ~/catkin_ws/devel/setup.bash
``` 

# To run

* Run detector node. For testing (until detector node is publishing the bbox messages), run [Prerit's code](https://github.com/bostondiditeam/MV3D/tree/master/utils/ROSviz) for tracklet vizualization.

* Then run tracker node:

```rosrun tracker tracker.py```

---