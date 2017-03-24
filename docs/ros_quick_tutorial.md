# Navigating the ROS Filesystem
**Explain what is ROS package and manifests,how to find package(rospack find), change directory to a package location (roscd),and list package directory contents (rosls).**

**Packages**: software organization unit of ROS code. Each package can contain libraries, executables, scripts or other artifacts.

**Manifests (package.xml)**: A manifest is a description of a package. It serves to define dependencies between packages and to capture meta information about the package like version, maintainer, license, etc...

## Find package (rospack find)
$ rospack find [package_name]
```
$ rospack find roscpp
/opt/ros/indigo/share/roscpp
```

## Change directory to a package location (roscd)
$ roscd [locationame[/subdir]]
```
$ roscd roscpp
/opt/ros/indigo/share/roscpp$
```

## List package directory contents (rosls)
$ rosls
```
/opt/ros/indigo/share/roscpp$ rosls
cmake  msg  package.xml  rosbuild  srv
```

# Creating and building a ROS package
**Use Catkin, a low-level build system macros and infrastructure for ROS, to create and build ROS package.**

Catkin package requirement :
* package.xml (it must be catkin compliant, will be discussed later...)
* CMakeLists.txt
* No more than one package in each folder

For example, a simplest catkin package may look like
```
my_package/
  CMakeLists.txt
  package.xml
```

First, we can create a catkin workspace.  A catkin workspace is a folder where you modify, build, and install catkin packages.
For example, a workspace looks like this:
```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
      
  build/                  -- BUILD SPACE
  devel/                  -- DEVELOPMENT SPACE (set by CATKIN_DEVEL_PREFIX)
  install/                -- INSTALL SPACE (set by CMAKE_INSTALL_PREFIX)
```


## Create a workspace and initialize it (catkin_make)

$ catkin_make

We create a workspace named 'catkin_ws' with an empty 'src' folder inside and initialize it with catkin_make.
```
$ source /opt/ros/indigo/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
~/catkin_ws$ ls
src

~/catkin_ws$ catkin_make
Base path: /home/raymond/catkin_ws
Source space: /home/raymond/catkin_ws/src
Build space: /home/raymond/catkin_ws/build
... 
~/catkin_ws$ ls
build deval src

```
**Note to Mac users:** If you encounter an `Invoking "cmake" failed` error, you may need to install `build-essential` in your ROS environment with the proper compilers.  From your ROS prompt, type the following:
```
sudo apt-get update
sudo apt-get install build-essential
```
Once installation completes, re-run the `catkin_make` command.

## Create a package (catkin_create_pkg) 

$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]

We create a package called 'beginner_tutorials' which depends on std_msgs, roscpp, and rospy.
```
~/catkin_ws$ cd src
~/catkin_ws/src$ ls
CMakeLists.txt

~/catkin_ws/src$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp    
Created file beginner_tutorials/CMakeLists.txt
Created file beginner_tutorials/package.xml
Created folder beginner_tutorials/include/beginner_tutorials
Created folder beginner_tutorials/src
Successfully created files in /home/raymond/catkin_ws/src/beginner_tutorials. Please adjust the values in package.xml.

~/catkin_ws/src$ ls
beginner_tutorials  CMakeLists.txt
```

## Build a package (catkin_make)

$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]    # (in catkin workspace)

$ catkin_make install  # (optionally)

```
~/catkin_ws/src$ cd ..
~/catkin_ws$ catkin_make
...

~/catkin_ws$ . ~/catkin_ws/devel/setup.bash  #To add the workspace to your ROS environment you need to source the generated setup file

```
Note whenever you build a new package, you should source the generated setup file to update your environment !


## Modify package.xml which contains meta information
We can modify maintainer, license, build_depend or run_depend package in package.xml.

```
~/catkin_ws$ cd src/beginner_tutorials
~/catkin_ws/src/beginner_tutorials$ ls
CMakeLists.txt  include  package.xml  src

~/catkin_ws/src/beginner_tutorials$ vim package.xml      #Just for simplicity, package.xml shows here without comments and unused tags. You may modify package.xml if needed.

<?xml version="1.0"?>
<package>
   <name>beginner_tutorials</name>
   <version>0.1.0</version>
   <description>The beginner_tutorials package</description>
          
   <maintainer email="you@yourdomain.tld">Your Name</maintainer>
   <license>BSD</license>
   <url type="website">http://wiki.ros.org/beginner_tutorials</url>
   <author email="you@yourdomain.tld">Jane Doe</author>
   
   <buildtool_depend>catkin</buildtool_depend>

   <build_depend>roscpp</build_depend>
   <build_depend>rospy</build_depend>
   <build_depend>std_msgs</build_depend>
   
   <run_depend>roscpp</run_depend>
   <run_depend>rospy</run_depend>
   <run_depend>std_msgs</run_depend>
</package>

```


# Running a ROS Node
**Node is just an excutable file within a ROS package. Here, we initialize "master" service (roscore) to handle nodes communication first and then run a node (rosrun). You may check running nodes (rosnode list), their details (rosnode info) and check if the node is up (rosnode ping)**


**Nodes**: an executable file within a ROS package. nodes use a ROS client library to communicate with other nodes. Nodes can publish or subscribe to a Topic. Nodes can also provide or use a Service.

**Client libraries**: allow nodes written in different programming languages to communicate:
* rospy = python client library
* roscpp = c++ client library

**Messages**: ROS data type used when subscribing or publishing to a topic.

**Topics**: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.

**Master**: Name service for ROS (i.e. helps nodes find each other)

**rosout**: ROS equivalent of stdout/stderr

**roscore**: Master + rosout + parameter server (parameter server will be introduced later)


## Initialize with creating a ROS master which manage communications between nodes (roscore)

$ roscore

```
$ roscore
... 
auto-starting new master
process[master]: started with pid [26684]
ROS_MASTER_URI=http://raymond-VirtualBox:11311/

setting /run_id to 77c7dd08-0fec-11e7-a32b-080027b26b86
process[rosout-1]: started with pid [26698]
started core service [/rosout]
```

## List running ROS node (rosnode list) and check node's information (rosnode info [/node_name])

$ rosnode list

$ rosnode info [/node_name]

We leave the terminal where roscore running, create another terminal and check running node and information via rosnode list and rosnode info.
```
# leave the terminal roscore running and create another terminal ...
$ rosnode list  
/rosout           # node 'rosout' is always running as it collects and logs nodes' debugging output.

$ rosnode info /rosout
Node [/rosout]
Publications:
 * /rosout_agg [rosgraph_msgs/Log]

Subscriptions:
 * /rosout [unknown type]

Services:
 * /rosout/set_logger_level
 * /rosout/get_loggers
 
contacting node http://machine_name:54614/ ...
Pid: 5092

```

## Run a ROS node from package (rosrun) and check if node is up (rosnode ping)

$ rosrun [package_name] [node_name]

```
$ rosrun turtlesim turtlesim_node     # this create a running node "turtlesim_node" with default name "turtlesim" from package "turtlesim" and will pop up a window with a turtle inside

$ rosnode list    # check running nodes, we find turtlesim_node
/rosout
/turtlesim

# close the windown and we can give the node user-defined name "my_turtle" as follows
$ rosrun turtlesim turtlesim_node __name:=my_turtle

$ rosnode list
/rosout
/my_turtle

#check if node my_turtle is up
$ rosnode ping my_turtle  
rosnode: node is [/my_turtle]
pinging /my_turtle with a timeout of 3.0s
xmlrpc reply from http://aqy:42235/     time=1.152992ms
xmlrpc reply from http://aqy:42235/     time=1.120090ms
...
```

