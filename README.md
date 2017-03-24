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

### Find package: 
$ rospack find [package_name]
```
$ rospack find roscpp
/opt/ros/indigo/share/roscpp
```

### Change directory to a package location:
$ roscd [locationame[/subdir]]
```
$ roscd roscpp
/opt/ros/indigo/share/roscpp$
```

### List package directory contents
$ rosls
```
/opt/ros/indigo/share/roscpp$ rosls
cmake  msg  package.xml  rosbuild  srv
```

## Creating and building a ROS package
**Use Catkin, a low-level build system macros and infrastructure for ROS, to create and build ROS package. **

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


### create a workspace (catkin_ws), initialize it with catkin_make

$ catkin_make

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

### create a package called 'beginner_tutorials' which depends on std_msgs, roscpp, and rospy

$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]

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

### build a package

$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]    # (in catkin workspace)

$ catkin_make install  # (optionally)

```
~/catkin_ws/src$ cd ..
~/catkin_ws$ catkin_make
...

~/catkin_ws$ . ~/catkin_ws/devel/setup.bash  #To add the workspace to your ROS environment you need to source the generated setup file

```

### Modify package.xml which contains meta information, has been tailored to your package, you can modify maintainer, license, build_depend or run_depend package here.

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



