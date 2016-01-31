#!/bin/bash
echo "Installing udev and init.d rules"
sudo cp $(catkin_find --share rospilot share/etc/rospilot.rules) /etc/udev/rules.d/
sudo cp $(catkin_find --share rospilot share/etc/init.d/rospilot) /etc/init.d/
sudo chmod +x /etc/init.d/rospilot
message="Enabling rospilot service. \
Uninstall it with 'update-rc.d -f rospilot remove'"

echo "$(tput setaf 1)${message}$(tput sgr 0)"
sudo update-rc.d rospilot defaults
