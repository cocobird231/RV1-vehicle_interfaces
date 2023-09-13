#!/usr/bin/bash

# Install dependencies
ubuntu_ver=$(lsb_release -r | grep Release | grep -Po '[\d.]+')
if [ "$ubuntu_ver" == "18.04" ]
then
    echo "Add PPA: 'team-xbmc'"
    sudo add-apt-repository ppa:team-xbmc/ppa -y
    sudo apt update
fi
sudo apt install nlohmann-json3-dev -y