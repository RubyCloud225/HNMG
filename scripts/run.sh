#!/bin/bash

# Allow Docker to use the Mac display via XQuartz

xhost +localhost

docker build -t hnmg .

docker run --rm \ 
    -e DISPLAY=host.docker.internel:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    hnmg

# Revoke display access after run
xhost -localhost
