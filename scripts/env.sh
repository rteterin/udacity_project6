#!/bin/bash

SCRIPT_DIR=$(readlink -f $(dirname "$0"))

export TURTLEBOT_GAZEBO_WORLD_FILE=$(readlink -f "$SCRIPT_DIR/../world/world_001.world")
export TURTLEBOT_GAZEBO_MAP_FILE=$(readlink -f "$SCRIPT_DIR/../map/map.yaml")

echo "World file: $TURTLEBOT_GAZEBO_WORLD_FILE"
echo "Map file: $TURTLEBOT_GAZEBO_MAP_FILE"
