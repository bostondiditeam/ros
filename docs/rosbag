# Overview

The rosbag package provides a command-line tool for working with bags as well as code APIs for reading/writing bags in [C++](http://wiki.ros.org/rosbag/Code%20API#cpp_api) and [Python](http://wiki.ros.org/rosbag/Code%20API#py_api).  Since Python is a more prevalent skill on the Boston Didi team, we may consider using it whenever C++ is not required.  [rosbag Overview on YouTube (1:10)](https://youtu.be/pwlbArh_neU)

A [bag](http://wiki.ros.org/Bags) is a file format in ROS for storing ROS message data. Bags -- so named because of their .bag extension -- have an important role in ROS, and a variety of tools have been written to allow you to store, process, analyze, and visualize them.
* Bags are typically created by a tool like rosbag, which subscribe to one or more ROS topics, and store the serialized message data in a file as it is received. These bag files can also be played back in ROS to the same topics they were recorded from, or even remapped to new topics.
* Bags are the primary mechanism in ROS for data logging, which means that they have a variety of offline uses. Researchers have used the bag file toolchain to record datasets, then visualize, label them, and store them for future use. Bag files have also been used to perform long-term hardware diagnostics logging for the PR2 robot.


# Python API
Found [here](http://docs.ros.org/diamondback/api/rosbag/html/python/).

Example usage for write:

```
import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('test.bag', 'w')

try:
    str = String()
    str.data = 'foo'

    i = Int32()
    i.data = 42

    bag.write('chatter', str)
    bag.write('numbers', i)
finally:
    bag.close()
```
Example usage for read:

```
import rosbag
bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
    print msg
bag.close()
```

# GUI plugin
[rqt_bag](http://wiki.ros.org/rqt_bag) provides a GUI plugin for displaying and replaying ROS bag files.