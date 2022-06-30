#!/bin/bash

BASEPATH=$(cd `dirname $0`; pwd)
echo $BASEPATH
sleep 5
gnome-terminal -x bash -c "rosbag play $BASEPATH/../bag/record1.bag"

