#!/bin/bash

ROS_MASTER_URI=http://cortex:11311 rosrun image_view image_view image:=/camera/image_raw _image_transport:=compressed
