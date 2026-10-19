#!/bin/bash
set -eo pipefail

source /home/user/concert-docker-env.bash
exec "$@"
