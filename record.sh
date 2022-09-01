#! /bin/bash
mkdir /home/dualarm/handover-system/bags/

rosbag record -o /home/dualarm/handover-system/bags/ \
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
