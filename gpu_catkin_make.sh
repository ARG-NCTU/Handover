#! /bin/bash

# CATKIN_IGNORE - mm-core 
# touch ./catkin_ws/src/mm-core/arm_operation/abb_control/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/arm_operation/dual_arm_operation/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/arm_operation/ur5_arm_control/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/bin_picking/ddqn/grasp_suck/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/bin_picking/ddqn/visualization/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/bin_picking/ddqn/visual_system/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/pose_estimate/electronics_pick_and_assemble/pose_estimation/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/pose_estimate/electronics_pick_and_assemble/pose_estimate_and_pick/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/semantic_segmentation/electronics_pick_and_assemble/object_detection/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/semantic_segmentation/electronics_pick_and_assemble/object_detection_lily/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/system_fsm/barcode_flatten_system/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/system_fsm/pick_and_place_system/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/system_fsm/semantic_system/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/tools/CATKIN_IGNORE
# touch ./catkin_ws/src/mm-core/semantic_segmentation/barcode_detection/CATKIN_IGNORE  

# CATKIN_IGNORE - algorithm 
# touch ./catkin_ws/src/algorithm/CATKIN_IGNORE

# python3 cv_bridge
catkin_make --pkg vision_opencv -C ./catkin_ws \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

catkin_make --pkg geometry2 -C ./catkin_ws \
            --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

catkin_make -C ./catkin_ws
