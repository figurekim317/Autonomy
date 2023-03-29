#!/bin/bash
source /opt/ros/eloquent/setup.bash

wait

source /home/linkxavier/ros2_ws/install/setup.bash

wait

/opt/ros/eloquent/bin/ros2 launch object_detection yolov5.launch.py
