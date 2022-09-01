#! /bin/bash

if [ "$1" ]; then
    echo "ROS MASRER $1"
    export ROS_MASTER_URI=http://$1:11311
else
    # echo "ROS MASRER 127.0.0.1"
    # export ROS_MASTER_URI=http://127.0.0.1:11311
    echo "ROS MASRER 192.168.50.185"
    export ROS_MASTER_URI=http://192.168.50.185:11311
fi

if [ "$2" ]; then
    echo "ROS IP $2"
    export ROS_IP=$2
else
    # echo "ROS IP 127.0.0.1"
    # export ROS_IP=127.0.0.1
    echo "ROS IP 192.168.50.161"
    export ROS_IP=192.168.50.161
fi

source /opt/ros/melodic/setup.bash
source ./catkin_ws/devel/setup.bash

# source ~/.bashrc
source ~/Handover/catkin_ws/devel/setup.bash

echo "pick and place environment"
