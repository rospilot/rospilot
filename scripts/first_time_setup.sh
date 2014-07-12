#!/bin/bash
message="Enabling rospilot service. \
Uninstall it with 'update-rc.d -f rospilot remove'"

echo "$(tput setaf 1)${message}$(tput sgr 0)"
sudo update-rc.d rospilot defaults
