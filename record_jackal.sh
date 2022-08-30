#! /bin/bash
mkdir /home/kl/Pick-and-Place-with-RL/exp2/

# rosbag record -o /home/kl/Pick-and-Place-with-RL/piyan/ --duration=1 \
rosbag record -o /home/kl/Pick-and-Place-with-RL/exp2/ \
    /camera_left/aligned_depth_to_color/image_raw/compressed \
    /camera_left/aligned_depth_to_color/camera_info \
    /camera_left/color/camera_info \
    /camera_left/color/image_raw/compressed \
    /camera_right/aligned_depth_to_color/image_raw/compressed \
    /camera_right/aligned_depth_to_color/camera_info \
    /camera_right/color/camera_info \
    /camera_right/color/image_raw/compressed \
    /camera_mid/aligned_depth_to_color/image_raw/compressed \
    /camera_mid/aligned_depth_to_color/camera_info \
    /camera_mid/color/camera_info \
    /camera_mid/color/image_raw/compressed \
    /handover_server/affordance_map_center_pcl \
    /tf \
    /tf_static
