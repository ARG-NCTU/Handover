#! /bin/bash

cwd=$PWD

{ read -r var1;} < <(yq -r .handover.nuc.ip_in $PWD/config/system-device.yaml)
{ read -r var2;} < <(yq -r .handover.workstation.ip_in $PWD/config/system-device.yaml)


echo "ROS MASRER" $var1
export ROS_MASTER_URI=http://$var1:11311

if [ "$1" == 'nuc' ]; then
    echo "ROS IP $var1"
    export ROS_IP=$var1
else
    echo "ROS IP $var2"
    export ROS_IP=$var2
fi

source /opt/ros/melodic/setup.bash
source ./catkin_ws/devel/setup.bash

# source ~/.bashrc
source ~/handover-system/catkin_ws/devel/setup.bash

echo "Handover environment"
