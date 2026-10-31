#!/bin/bash
# Sources the workspace, optionally starts a Fast DDS Discovery Server, then runs the given command.
#   START_DISCOVERY_SERVER=1      start "fastdds discovery" in the background (edge only)
#   DISCOVERY_SERVER_PORT=11811   its UDP port
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

if [ "${START_DISCOVERY_SERVER:-0}" = "1" ]; then
  fastdds discovery -i 0 -l 0.0.0.0 -p "${DISCOVERY_SERVER_PORT:-11811}" &
fi

exec "$@"
