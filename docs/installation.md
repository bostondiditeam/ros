# Installing ROS with Docker
Extracted and modified from [Getting started with ROS and Docker](http://wiki.ros.org/docker/Tutorials/Docker).

**Note:** These instructions apply to Mac OS and Linux.  Please modify
these instructions if your OS requires tweaks.

**Requirements:**  You will need to install Docker first and have it running before proceeding.  Please refer to the
 [Docker install instructions](https://docs.docker.com/engine/installation/) if you have not yet installed it.

## Step 1: Pull the ROS Docker image
Open up a Terminal window and type the following command:

```
docker pull ros:indigo
```

This only needs to be run once, or until you remove the `ros:indigo` Docker image.

You should see output similar to the following:

```
[Tue Mar 21 13:24] ROS ppoon <509>$ docker pull ros:indigo
indigo: Pulling from library/ros
30d541b48fc0: Pull complete
8ecd7f80d390: Pull complete
46ec9927bb81: Pull complete
2e67a4d67b44: Pull complete
7d9dd9155488: Pull complete
521da8c15ffa: Pull complete
90bdf6eb1ca5: Pull complete
f7f63520063b: Pull complete
639353f6de7b: Pull complete
1cca8d2707ad: Pull complete
8d81c3c32c52: Pull complete
7250d43156b8: Pull complete
958af582ec94: Pull complete
Digest: sha256:1bb1fad2c31d2dccb6048d44662450065c889983b4bcfb728cf0ad7bd76d706a
Status: Downloaded newer image for ros:indigo
```

## Step 2: Run the ROS Docker container
```
docker run -it ros:indigo
```

This will move you into an interactive session with the running container.  You should see a command prompt, like the following:


```
root@e99f472e43ca:/#
```
From here, it's basically as if you're in a new bash terminal separate from your host machine.

You will need to run `source /opt/ros/indigo/setup.bash` on every new shell you open to have access to the ROS commands, unless you add this statement to your `.bashrc` file.

Here's a quick way to do that (this only needs to be done once):

```
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
```

## Step 3: Start the roscore service

From the ROS prompt above, type:

```
roscore
```

You should see output similar to the following:

```
root@e97fa7fa281b:/# roscore
... logging to /root/.ros/log/4e92fa90-0e60-11e7-bc89-0242ac110004/roslaunch-e97fa7fa281b-87.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.
	
started roslaunch server http://e97fa7fa281b:37653/
ros_comm version 1.11.20
	
	
SUMMARY
========
	
PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.20
	
NODES
	
auto-starting new master
process[master]: started with pid [98]
ROS_MASTER_URI=http://e97fa7fa281b:11311/
	
setting /run_id to 4e92fa90-0e60-11e7-bc89-0242ac110004
process[rosout-1]: started with pid [111]
started core service [/rosout]
```

## Step 4: Open a new shell to issue ros commands

Assuming that you have already added the `source /opt/ros/<distro>/setup.bash` line to your `.bashrc` in Step 2 above, open a new shell and type `docker ps -l`.
You should see output similar to the following (Please scroll right to see all columns):

```
[Tue Mar 21 13:46] ROS ppoon <503>$ docker ps -l
CONTAINER ID        IMAGE               COMMAND                  CREATED             STATUS              PORTS               NAMES
e97fa7fa281b        ros:indigo          "/ros_entrypoint.s..."   10 minutes ago      Up 10 minutes                           frosty_allen
```

Please note the values under the "CONTAINER ID" and the "NAMES" columns.  You can use either of these values to attach to the running container that you started in the other shell.
Please type `docker exec -it <container id or name> bash`.  With the values above:

```
docker exec -it frosty_allen bash
```

You will attach to a running Docker container, and a ros prompt like the one above in Step 2 should appear.  At this point, you can issue ros commands.  Try:
```
rostopic list
```

You should see the following:

```
root@e97fa7fa281b:/# rostopic list
/rosout
/rosout_agg
```

