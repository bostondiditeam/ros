# Installing ROS on Ubuntu without Docker
Extracted and modified from [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).

**Note:** This is for installing [ROS Kinetic Kame](http://wiki.ros.org/kinetic), the tenth ROS distribution release, released May 23rd, 2016, on Ubuntu 16.04 LTS.

## Step 1: Setup your sources.list
Setup your computer to accept software from packages.ros.org. ROS Kinetic ONLY supports Wily (Ubuntu 15.10), Xenial (Ubuntu 16.04) and Jessie (Debian 8) for debian packages.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## Step 2: Set up your keys
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

## Step 3: Installation
First, make sure your Debian package index is up-to-date:
```
sudo apt-get update
```
Desktop-Full Install: (Recommended) : ROS, rqt, rviz, robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception
```
sudo apt-get install ros-kinetic-desktop-full
```

## Step 4: Initialize rosdep
Before you can use ROS, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.
```
sudo rosdep init
rosdep update
```

## Step 5: Environment setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:
```
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 6: Getting rosinstall
rosinstall is a frequently used command-line tool in ROS that is distributed separately. It enables you to easily download many source trees for ROS packages with one command.

To install this tool on Ubuntu, run:
```
sudo apt-get install python-rosinstall
```

## Verify
Follow Step 3 in the [installation guide with Docker](https://github.com/bostondiditeam/ros/blob/master/docs/installation.md) to verify correct installation.

Start roscore in one terminal,
```
roscore
```
Run a ROS command in another terminal, e.g.
```
rostopic list
```
You should see
```
root@e97fa7fa281b:/# rostopic list
/rosout
/rosout_agg
```
