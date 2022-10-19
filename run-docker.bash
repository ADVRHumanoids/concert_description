#!/bin/bash

nvidia-docker run --rm -it --gpus all \
 --env="DISPLAY=$DISPLAY" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 --volume="$HOME/.ssh:/home/user/.ssh:ro" \
 --name modular_description \
 arturolaurenzi/modular_description:latest \
 x-terminal-emulator
