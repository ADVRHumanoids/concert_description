#!/bin/bash
set -euo pipefail

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

docker build --tag edoardoromiti/concert_description:latest \
    -f "$DIR/Dockerfile" "$@" "$DIR/.."
