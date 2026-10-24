#!/usr/bin/env bash
set -eo pipefail

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${MODULE_SETUP:-/ws/install/setup.bash}"

exec "$@"
