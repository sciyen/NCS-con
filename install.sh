#!/usr/bin/env bash

echo Installing pycamera
sudo pip3 install "picamera[array]" && 

echo Installing requirements for opencv distrib &&
sudo apt-get install build-essential cmake pkg-config libjpeg-dev libtiff5-dev libjasper-dev libpng-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libfontconfig1-dev libcairo2-dev libgdk-pixbuf2.0-dev libpango1.0-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran libhdf5-dev libhdf5-serial-dev libhdf5-103 python3-pyqt5 python3-dev -y &&

echo Installing opencv distrib &&
sudo pip3 install opencv-contrib-python==4.5.3.56 &&

echo Installing opencv distrib &&
pip install numpy --upgrade &&
sudo pip3 install imutils pyyaml

if [ $? -eq 0 ]; then
   echo INSTALLATION SUCCEED
else
   echo INSTALLATION FAILED
fi
