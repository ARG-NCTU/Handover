#! /bin/bash

source /opt/ros/melodic/setup.bash
source ./catkin_ws/devel/setup.bash

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
