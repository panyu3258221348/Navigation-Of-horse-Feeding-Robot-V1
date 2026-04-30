#!/bin/bash
sudo cp ./50-rover.rules  /etc/udev/rules.d

echo " "
echo "Restarting udev"
service udev reload
sleep 2
service udev restart
