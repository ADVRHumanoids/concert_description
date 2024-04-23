#!/bin/bash

docker run --runtime nvidia --rm -it --gpus all \
 --env="NVIDIA_DRIVER_CAPABILITIES=all" \
 --env="DISPLAY=$DISPLAY" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 --volume="$HOME/.ssh:/home/user/.ssh:ro" \
 --name concert_description \
 arturolaurenzi/concert_description:latest \
 x-terminal-emulator
