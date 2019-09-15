#!/bin/bash

source $(dirname "$0")/env.sh

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "rosrun rviz rviz -d $SCRIPT_DIR/../rvizConfig/home_service.rviz" &
sleep 5
xterm -e "rosrun add_markers add_markers" &
sleep 5
xterm -e "rosrun pick_objects pick_objects"
