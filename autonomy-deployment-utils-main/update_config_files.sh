#!/bin/bash
sudoPW=neubility

DEPLOYMENT_REPOSITORY=autonomy-software-deployment
cd ~/$DEPLOYMENT_REPOSITORY
git pull
git submodule update --init --remote --recursive
rm -rf ~/ros2_ws/install/utils
cp -r ~/autonomy-software-deployment/utils ~/ros2_ws/install/utils

echo $sudoPW | sudo -S cp ~/ros2_ws/install/utils/fstab /etc/fstab
echo $sudoPW | sudo -S cp ~/ros2_ws/install/utils/99-usb-serial.rules /etc/udev/rules.d/
echo $sudoPW | sudo -S cp ~/ros2_ws/install/utils/10-cyclone-max.conf /etc/sysctl.d/
echo $sudoPW | sudo -S cp ~/ros2_ws/install/utils/chrony.conf /etc/chrony/
echo $sudoPW | sudo -S cp ~/ros2_ws/install/utils/daemon.json /etc/docker/
echo $sudoPW | sudo -S cp ~/ros2_ws/install/utils/prometheus.yaml /etc/monitoring
echo $sudoPW | sudo -S cp ~/ros2_ws/install/utils/filename.yaml /etc/monitoring
echo $sudoPW | sudo -S cp ~/ros2_ws/install/utils/docker-compose.yaml /etc/monitoring
