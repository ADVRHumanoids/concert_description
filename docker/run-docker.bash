#!/bin/bash

docker run --runtime nvidia --rm -it --gpus all \
 --env="NVIDIA_DRIVER_CAPABILITIES=all" \
 --env="__NV_PRIME_RENDER_OFFLOAD=1" \
 --env="__GLX_VENDOR_LIBRARY_NAME=nvidia" \
 --env="DISPLAY=$DISPLAY" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 --volume="$HOME/.ssh:/home/user/.ssh:ro" \
 --name concert_description \
 --publish=5003:5003 \
 edoardoromiti/concert_description:latest \
 x-terminal-emulator
