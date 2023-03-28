#!/bin/bash
sudoPW=neubility

echo $sudoPW | sudo -S /bin/cp /etc/skel/.bashrc ~/

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
echo "export CYCLONEDDS_URI=file:///home/linkxavier/robot_cfg/cyclonedds/cyclonedds.xml" >> ~/.bashrc
