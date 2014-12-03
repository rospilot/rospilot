/*********************************************************************
 *
 * Copyright 2012 the original author or authors.
 * See the NOTICE file distributed with this work for additional
 * information regarding copyright ownership.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *********************************************************************/
#ifndef ROSPILOT_USB_CAMERA_H
#define ROSPILOT_USB_CAMERA_H

#include<stdio.h>
#include<iostream>

#include<ros/ros.h>
#include<std_srvs/Empty.h>
#include<sensor_msgs/fill_image.h>
#include<sensor_msgs/CompressedImage.h>
#include<usb_cam/usb_cam.h>

#include<base_camera.h>

class UsbCamera : public BaseCamera
{
public:
    bool getLiveImage(sensor_msgs::CompressedImage *image) override
    {
        image->format = "jpeg";
        usb_cam_camera_grab_mjpeg(&(image->data));
        image->header.stamp = ros::Time::now();
        return true;
    }
    
    bool captureImage(sensor_msgs::CompressedImage *image) override
    {
        getLiveImage(image);
    }

public:
    UsbCamera(std::string device, int width, int height, int framerate)
    {
        usb_cam_camera_image_t *image;

        ROS_INFO("device: %s", device.c_str());
        image = usb_cam_camera_start(device.c_str(),
                                       IO_METHOD_MMAP,
                                       PIXEL_FORMAT_MJPEG,
                                       width,
                                       height,
                                       framerate);
        if (image == nullptr) {
            usleep(1000000);
            image = usb_cam_camera_start(device.c_str(),
                                           IO_METHOD_MMAP,
                                           PIXEL_FORMAT_MJPEG,
                                           width,
                                           height,
                                           framerate);
        }
        if (image != nullptr) {
            free(image);
        }
    }

    ~UsbCamera()
    {
        usb_cam_camera_shutdown();
    }
};

#endif
