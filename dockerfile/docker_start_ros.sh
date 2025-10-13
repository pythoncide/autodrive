#!/bin/bash

xhost +local:root

docker run -dit \
	--name IntelPi \
	--privileged \
	--restart always \
	--network host \
	-e DISPLAY=:0 \
	-e ROS_DOMAIN_ID=3 \
	-e ROS_DOMAIN_NAME=surim \
	-v /dev:/dev \
	-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v ${HOME}/docker/shared:/shared \
	ros:humble-export \
	tail -f /dev/null
