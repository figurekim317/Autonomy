#!/bin/bash
source /opt/ros/eloquent/setup.bash

wait

source /home/linkxavier/ros2_ws/install/setup.bash

wait

robot_cfg_string=$(cat ~/robot_cfg/camera_params.yaml)
substring="Only Perception"

if [[ "$robot_cfg_string" != *"$substring"* ]]; then
    cat `ros2 pkg prefix object_detection`/share/object_detection/params/camera_params.yaml >> /home/linkxavier/robot_cfg/camera_params.yaml
else
    echo "Already adapted perception camera params"
fi
