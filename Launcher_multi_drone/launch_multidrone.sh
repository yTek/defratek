#!/bin/bash

cd $HOME/bebop_ws/

echo "Launching files"

for param in $(seq 1 5);
do
  if ls src/bebop_autonomy/bebop_driver/launch/ | grep "bebop_node_bebop$param.launch"; then
    roslaunch bebop_driver bebop_node_bebop$param.launch &
    sleep 2
  else
    echo "no match $param"
  fi
done

