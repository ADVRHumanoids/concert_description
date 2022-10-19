#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

cd $DIR

docker build --tag arturolaurenzi/modular_description:latest . -f $DIR/Dockerfile