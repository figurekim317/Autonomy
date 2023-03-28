#!/bin/bash

DEPLOYMENT_REPOSITORY=autonomy-software-deployment
sudoPW=neubility

cd ~/$DEPLOYMENT_REPOSITORY

git pull --recurse-submodules
git submodule update --init --remote --recursive
tag=`git describe`
branch=`git rev-parse --abbrev-ref HEAD`

gitmsg=$(git pull 2>&1)
if [[ $gitmsg =~ "Could not resolve host: github.com" ]]; then
    echo "cannot connect network"
    new_hash=`git rev-parse HEAD`
elif [[ $gitmsg =~ "error" ]] || [[ $gitmsg =~ "fatal" ]]; then
    echo "git error detected!!"
    ./utils/reset_git.sh
fi

./utils/reset_crontab.sh
./utils/reset_bashrc.sh
./utils/reset_netrc.sh
./utils/folder_checker.sh
./utils/update_config_files.sh
./utils/pip_check.sh
./utils/install_requirements.sh
./utils/pull_docker_images.sh

./utils/disable_services.sh
rm -rf ~/ros2_ws/install
cp -r ~/$DEPLOYMENT_REPOSITORY ~/ros2_ws/install
./utils/update_services.sh

today=$(date)
echo binary updated at $today : $branch/$tag >> /home/linkxavier/version_update.log 
echo $sudoPW | sudo -S reboot
