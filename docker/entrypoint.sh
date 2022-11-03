#!/bin/bash
set -e

source /opt/ros/humble/setup.sh
source ~/ur_ws/install/setup.sh

exec "$@"