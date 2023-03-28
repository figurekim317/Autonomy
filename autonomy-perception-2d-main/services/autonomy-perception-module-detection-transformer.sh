#!/bin/bash
source /opt/ros/eloquent/setup.bash

wait

source /home/linkxavier/ros2_ws/install/setup.bash

wait

FileRoot=~/robot_cfg
File=base_link_ground_to_cam_matrices.yaml
DefaultFile=default_transform_params.yaml

ln -f `ros2 pkg prefix perception_2d`/share/perception_2d/params/$DefaultFile "$FileRoot/$File"

/opt/ros/eloquent/bin/ros2 launch perception_2d detection_transformer.launch.py
