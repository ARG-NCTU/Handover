#! /bin/bash

cwd=$PWD

{ read -r var1;} < <(yq -r .handover.nuc.ip_in $PWD/config/system-device.yaml)
{ read -r var2;} < <(yq -r .handover.workstation.ip_in $PWD/config/system-device.yaml)




if [ "$1" == 'nuc' ]; then
    echo "ROS MASRER" $var1
    export ROS_MASTER_URI=http://$var1:11311

    echo "ROS IP $var1"
    export ROS_IP=$var1
elif [ "$1" == 'ws' ]; then
    echo "ROS MASRER" $var1
    export ROS_MASTER_URI=http://$var1:11311

    echo "ROS IP $var2"
    export ROS_IP=$var2
else
    echo "ROS MASRER 127.0.0.1"
    export ROS_MASTER_URI=http://127.0.0.1:11311

    echo "ROS IP 127.0.0.1"
    export ROS_IP=127.0.0.1
fi

source /opt/ros/melodic/setup.bash
source ./catkin_ws/devel/setup.bash

# source ~/.bashrc
source ~/handover-system/catkin_ws/devel/setup.bash

echo "Handover environment"
