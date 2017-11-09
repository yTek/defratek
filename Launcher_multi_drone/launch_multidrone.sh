#!/bin/bash

cd $HOME/bebop_ws/

echo "Creating $# lauch files"

var=0
for param in "$@";
do
 var=$((var+1))
 cp $HOME/bebop_ws/src/bebop_autonomy/bebop_driver/launch/bebop_node_basic.launch $HOME/bebop_ws/src/bebop_autonomy/bebop_driver/launch/bebop_node_bebop$var.launch
 sed -i "s/BEBOP_NAME/bebop$var/g" $HOME/bebop_ws/src/bebop_autonomy/bebop_driver/launch/bebop_node_bebop$var.launch
 sed -i "s/IP_BEBOP/$param/g" $HOME/bebop_ws/src/bebop_autonomy/bebop_driver/launch/bebop_node_bebop$var.launch
 echo "File $var ready"
done

for param2 in $(seq 1 $#);
do
 echo "roslaunch bebop_driver bebop_node_bebop$param2.launch &"
 sleep 1
done

