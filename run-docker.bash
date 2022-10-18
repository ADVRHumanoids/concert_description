#!/bin/bash

set -e

nvidia-docker run --rm -it --gpus all \
 --env="DISPLAY=$DISPLAY" \
 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 --name modular_description \
 modular_description:main \
 bash

