veh=racecar

mkdir /home/argsubt/racecar/bags/$(date +%m%d_%H%M)

rosbag record -o /home/argsubt/racecar/bags/$(date +%m%d_%H%M)/subt --split --size=2048 \
    /tf \
    /tf_static \
    /$veh/camera/color/camera_info \
    /$veh/camera/color/image_raw \
    /$veh/camera/aligned_depth_to_color/image_raw \
    /$veh/camera/aligned_depth_to_color/camera_info \
    /$veh/joy \
    /$veh/cmd_vel \
    /$veh/brushless_command \
    /$veh/scan \
    /$veh/back_left/ranges \
    /$veh/rssi_neighbour \
