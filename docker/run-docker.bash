#!/bin/bash
set -euo pipefail

IMAGE="${IMAGE:-ghcr.io/advrhumanoids/concert_description}"

docker run --runtime nvidia --rm -it --gpus all \
 --env="NVIDIA_DRIVER_CAPABILITIES=all" \
 --env="__NV_PRIME_RENDER_OFFLOAD=1" \
 --env="__GLX_VENDOR_LIBRARY_NAME=nvidia" \
 --env="DISPLAY=$DISPLAY" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 --volume="$HOME/.ssh:/home/user/.ssh:ro" \
 --name concert_description \
 --publish=5003:5003 \
 "$IMAGE:latest" \
 x-terminal-emulator
