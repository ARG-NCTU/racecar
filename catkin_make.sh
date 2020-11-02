#! /bin/bash

# python3 openCV 
catkin_make --pkg vision_opencv -C ./catkin_ws \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
    -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so

# robotx
#catkin_make --pkg robotx_msgs -C ./catkin_ws

# subt
#catkin_make --pkg apriltags2_ros -C ./catkin_ws
catkin_make --pkg realsense2_camera -C ./catkin_ws
catkin_make --pkg cloud_msgs -C ./catkin_ws
catkin_make --pkg subt_msgs -C ./catkin_ws
#catkin_make --pkg robotx_msgs -C ./catkin_ws
#catkin_make --pkg anchor_measure -C ./catkin_ws
#catkin_make --pkg sound_localize -C ./catkin_ws

# all
catkin_make -C ./catkin_ws
