#!/usr/bin/bash

# require root permission
if [ $UID -ne 0 ]; then
    echo "Execute me as root!"
    exit 1
fi

# create udev rule
echo 'KERNEL=="ttyACM*",ATTRS{idVendor}=="0483",ATTRS{idProduct}=="5740",MODE:="0666",GROUP:="dialout",SYMLINK+="boardc_imu"' > /etc/udev/rules.d/boardc_imu.rules

service udev reload
sleep 2
service udev restart