#!/bin/bash

SCRIPT_DIR=$(readlink -f $(dirname "$0"))

export TURTLEBOT_GAZEBO_WORLD_FILE="$SCRIPT_DIR/../world/home.world"
export TURTLEBOT_GAZEBO_MAP_FILE="$SCRIPT_DIR/../map/map.yaml"
