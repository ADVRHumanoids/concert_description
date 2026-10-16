#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

docker build --tag edoardoromiti/concert_description-ros1-legacy:latest . -f $DIR/Dockerfile "$@"