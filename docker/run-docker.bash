#!/bin/bash
set -euo pipefail

IMAGE="${IMAGE:-ghcr.io/advrhumanoids/concert_description}"

docker run --runtime nvidia --rm -it --gpus all \
 --env="NVIDIA_DRIVER_CAPABILITIES=all" \
 --env="DISPLAY=$DISPLAY" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 --volume="$HOME/.ssh:/home/user/.ssh:ro" \
 --name concert_description \
 --publish=5003:5003 \
 "$IMAGE:latest" \
 x-terminal-emulator
