#!/bin/bash


gnome-terminal --title="spark_control" --geometry 34x10+63+305 -- bash -c "rosrun spark_teleop spark_teleop_node 0.14 0.5"

