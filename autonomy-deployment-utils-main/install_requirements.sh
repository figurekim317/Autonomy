#!/bin/bash
sudoPW=neubility
DEPLOYMENT_REPOSITORY=autonomy-software-deployment
DIRCETORY_FOR_INSTALL_FILES=$HOME/install_temp

i=1
while read line || [ -n "$line" ] ; do
  ((i+=1))
  REQUIRED_PKG=$line
  PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $REQUIRED_PKG|grep "install ok installed")
  echo Checking for $REQUIRED_PKG: $PKG_OK
  if [ "" = "$PKG_OK" ]; then
    echo "No $REQUIRED_PKG. Setting up $REQUIRED_PKG."
    echo $sudoPW | sudo -S apt --yes install $REQUIRED_PKG
  fi

done < ~/$DEPLOYMENT_REPOSITORY/utils/requirements.txt

## install wireguard using deb
[ 0 -eq $(dpkg-query -W --showformat='${Status}\n' wireguard-dkms | grep 'install ok installed' | wc -l) ] && echo $sudoPW | sudo -S dpkg -i ~/$DEPLOYMENT_REPOSITORY/utils/wireguard-dkms_1.0.20201112-1~18.04.4_all.deb || echo 'install ok installed'
[ 0 -eq $(dpkg-query -W --showformat='${Status}\n' wireguard-tools | grep 'install ok installed' | wc -l) ] && echo $sudoPW | sudo -S dpkg -i ~/$DEPLOYMENT_REPOSITORY/utils/wireguard-tools_1.0.20200513-1~18.04.2_arm64.deb || echo 'install ok installed'
[ 0 -eq $(dpkg-query -W --showformat='${Status}\n' wireguard | grep 'install ok installed' | wc -l) ] && echo $sudoPW | sudo -S dpkg -i ~/$DEPLOYMENT_REPOSITORY/utils/wireguard_1.0.20200513-1~18.04.2_all.deb || echo 'install ok installed'

## update docker using official script.
echo $sudoPW | sudo -S sh ~/$DEPLOYMENT_REPOSITORY/utils/get-docker.sh --update
echo $sudoPW | sudo usermod -aG docker $USER
docker plugin install mochoa/s3fs-volume-plugin --alias s3fs --grant-all-permissions --disable

## install dependancy with binary
mkdir -p $DIRCETORY_FOR_INSTALL_FILES
curl "https://awscli.amazonaws.com/awscli-exe-linux-aarch64.zip" -o "$DIRCETORY_FOR_INSTALL_FILES/awscliv2.zip"
unzip -o $DIRCETORY_FOR_INSTALL_FILES/awscliv2.zip -d $DIRCETORY_FOR_INSTALL_FILES
echo $sudoPW | sudo bash $DIRCETORY_FOR_INSTALL_FILES/aws/install

## remove install files
rm -rf $DIRCETORY_FOR_INSTALL_FILES
