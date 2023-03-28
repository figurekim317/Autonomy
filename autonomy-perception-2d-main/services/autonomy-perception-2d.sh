#!/bin/bash
source /opt/ros/eloquent/setup.bash

wait

source /home/linkxavier/ros2_ws/install/setup.bash

wait

/opt/ros/eloquent/bin/ros2 launch perception_2d perception_2d.launch.py
