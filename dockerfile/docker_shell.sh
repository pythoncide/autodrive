#!/bin/bash

xhost +local:root

docker exec \
  -it \
  -u ubuntu \
  -w /home/ubuntu \
  -e need_compile=false \
  -e DEPTH_CAMERA_TYPE=ascamera \
  -e MACHINE_TYPE=MentorPi_Mecanum \
  IntelPi \
  /bin/zsh -lc '
    set -e
    [[ -f /opt/ros/humble/setup.zsh ]] && source /opt/ros/humble/setup.zsh
    [[ -f /home/ubuntu/ros2_ws/install/setup.zsh ]] && source /home/ubuntu/ros2_ws/install/setup.zsh

    cd /home/ubuntu/ros2_ws

    set +e
    ros2 launch example self_driving.launch.py start:=true only_line_follow:=false
    set -e

    exec /bin/zsh
  '
