#!/bin/bash

cd $HOME/bebop_ws/

echo "Launch the 2 driver for both drone"
roslaunch bebop_driver bebop_node_Billy.launch &
sleep 1
roslaunch bebop_driver bebop_node_Alfred.launch &

