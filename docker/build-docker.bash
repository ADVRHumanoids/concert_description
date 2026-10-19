#!/bin/bash
set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
IMAGE="${IMAGE:-ghcr.io/advrhumanoids/concert_description}"

docker build --tag "$IMAGE:latest" \
    -f "$DIR/Dockerfile" "$@" "$DIR/.."
