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

#include "usb_camera.h"

#include<fcntl.h>
#include<sys/ioctl.h>
extern "C" {
#include <linux/videodev2.h>
}

#include<ros/ros.h>
#include<rospilot_deps/usb_cam.h>

int UsbCamera::getWidth()
{
    return width;
}

int UsbCamera::getHeight()
{
    return height;
}

bool UsbCamera::getLiveImage(sensor_msgs::CompressedImage *image)
{
    image->format = "jpeg";
    usb_cam_camera_grab_mjpeg(&(image->data));
    image->header.stamp = ros::Time::now();
    return true;
}

bool UsbCamera::captureImage(sensor_msgs::CompressedImage *image)
{
    getLiveImage(image);
}

UsbCamera::UsbCamera(std::string device, int width, int height, int framerate)
{
    usb_cam_camera_image_t *image;

    // Adjust height and width to something this camera supports
    int fd;
    v4l2_format format;
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if ((fd = open(device.c_str(), O_RDONLY)) == -1){
        ROS_FATAL("Can't open %s", device.c_str());
    }

    if (ioctl(fd, VIDIOC_G_FMT, &format) == -1) {
        ROS_FATAL("Can't read format from %s", device.c_str());
    }
    else {
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        ioctl(fd, VIDIOC_TRY_FMT, &format);
        width = format.fmt.pix.width;
        height = format.fmt.pix.height;
    }
    close(fd);

    this->width = width;
    this->height = height;

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

UsbCamera::~UsbCamera()
{
    usb_cam_camera_shutdown();
}
