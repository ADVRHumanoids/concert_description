#!/bin/bash

SCRIPT_DIR=$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)
cd $SCRIPT_DIR
mkdir -p /tmp/.xbot2_concert_isaac
docker compose up -d ${1:-sim} --no-recreate
docker compose exec ${1:-sim} bash -i /workspace/concert_description/concert_isaac/scripts/entrypoint.bash