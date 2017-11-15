#!/bin/bash

cd $HOME/bebop_ws/

echo "Launching files"

for param in $(seq 1 5);
do
  if ls src/bebop_autonomy/bebop_driver/launch/ | grep "bebop_node_bebop$param.launch"; then
    echo "roslaunch bebop_driver bebop_node_bebop$param.launch &"
    sleep 1
  else
    echo "no match $param"
  fi
done

