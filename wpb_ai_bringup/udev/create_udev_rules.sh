#!/bin/bash

echo "***************"
echo "remap the device serial port(ttyUSBX) to ftdi"
echo "start copy ftdi.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpb_ai_bringup`/udev/ftdi.rules  /etc/udev/rules.d

echo "remap the device serial port(ttyUSBX) to rplidar"
echo "start copy rplidar.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpb_ai_bringup`/udev/rplidar.rules  /etc/udev/rules.d

echo "set kinect2 rules"
echo "start copy 90-kinect2.rules to  /etc/udev/rules.d/"
sudo cp `rospack find wpb_ai_bringup`/udev/90-kinect2.rules  /etc/udev/rules.d

echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
