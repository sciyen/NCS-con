#!/usr/bin/env bash

echo Installing basic developing tools
sudo apt install vim tmux git &&

echo Repo downloading &&
{
    if [[ ! -d ~/Project ]]; then 
        mkdir ~/Project && cd ~/Project
    fi
} &&

git clone https://github.com/NTU-NCS-lab/NCS-con.git

if [ $? -eq 0 ]; then
   echo INSTALLATION SUCCEED
else
   echo INSTALLATION FAILED
fi
