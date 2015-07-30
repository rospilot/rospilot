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
#include<third_party/usb_cam.h>
#include<rospilot/Resolution.h>
namespace rospilot {

int UsbCamera::getWidth()
{
    return width;
}

int UsbCamera::getHeight()
{
    return height;
}

rospilot::Resolutions UsbCamera::getSupportedResolutions()
{
    return resolutions;
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
    return getLiveImage(image);
}

bool tryResolution(int fd, Resolution resolution)
{
    v4l2_format format;
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &format) == -1) {
        ROS_FATAL("Can't read format from camera");
    }
    format.fmt.pix.width = resolution.width;
    format.fmt.pix.height = resolution.height;
    if (ioctl(fd, VIDIOC_TRY_FMT, &format) == -1) {
        ROS_FATAL("Can't try resolution");
    }
    return resolution.width == format.fmt.pix.width &&
        resolution.height == format.fmt.pix.height;
}

rospilot::Resolution res(int width, int height)
{
    rospilot::Resolution resolution;
    resolution.width = width;
    resolution.height = height;
    return resolution;
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
    // detect all supported resolutions
    std::vector<rospilot::Resolution> CANDIDATE_RESOLUTIONS = {
        // 16:9 resolutions
        res(854, 480), res(960, 544), res(1024, 576), res(1280, 720), res(1366, 768), res(1920, 1080), res(3840, 2160), res(7680, 4320),
        // 5:3 resolutions
        res(800, 480), res(1280, 768), 
        // 16:10 resolutions
        res(320, 200), res(1280, 800), res(1440, 900), res(1680, 1050), res(1920, 1200), res(2560, 1600),
        // 3:2 resolutions
        res(1152, 768), res(1280, 854), res(1440, 960), 
        // 4:3 resolutions
        res(320, 240), res(480, 360), res(640, 480), res(768, 576), res(800, 600),
        res(1024, 768), res(1280, 960), res(1400, 1050), res(1600, 1200), res(2048, 1536),
        // 5:4 resolutions
        res(352, 288), res(1280, 1024), res(2560, 2048)
    };
    for (rospilot::Resolution resolution : CANDIDATE_RESOLUTIONS) {
        if (tryResolution(fd, resolution)) {
            resolutions.resolutions.push_back(resolution);
        }
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

}
