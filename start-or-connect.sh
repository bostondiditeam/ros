#!/bin/bash


# run a container if does not exist,
# if container exists and running then attach it
# if container exists but stopped then start it

CONTAINER_NAME="didi"
IMAGE="didi-ros"
{
   sudo docker run -v $(pwd):/home/didi -it -p 8888:8888 --name=$CONTAINER_NAME  $IMAGE
} || {
   # if container is already running then connect to it
   sudo docker exec -i -t $CONTAINER_NAME  /bin/bash #by Name
} || {
   sudo docker start -i  -a  $CONTAINER_NAME
}

#echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

