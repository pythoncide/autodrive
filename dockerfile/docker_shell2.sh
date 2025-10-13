#!/bin/bash

xhost +local:root

docker exec \
    -it \
    -u ubuntu \
    -w /home/ubuntu \
    IntelPi \
    /bin/zsh
