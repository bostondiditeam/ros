Nodes are executable that can communicate with other processes using topics, services, or the Parameter Server. Using nodes in ROS provides us with fault tolerance and separates the code and functionalities, making the system simpler.

ROS has another type of node called nodelets. These special nodes are designed to run multiple nodes in a single process, with each nodelet being a thread (light process). This way, we avoid using the ROS network among them, but permit communication with other nodes. With that, nodes can communicate more efficiently, without overloading the network. Nodelets are especially useful for camera systems and 3D sensors, where the volume of data transferred is very high.

A node must have a unique name in the system. This name is used to permit the node to communicate with another node using its name without ambiguity. A node can be written using different libraries, such as roscpp and rospy; roscpp is for C++ and rospy is for Python. Throughout this book, we will use roscpp.

ROS has tools to handle nodes and give us information about it, such as rosnode. The rosnode tool is a command-line tool used to display information about nodes, such as listing the currently running nodes.
