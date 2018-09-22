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

uint32_t UsbCamera::getPixelFormat()
{
    if (pixelformat == V4L2_PIX_FMT_YUYV) {
        // Gets transcoded in getLiveImage()
        return V4L2_PIX_FMT_YUV420;
    }
    return pixelformat;
}

void convertYUYVToYUV420P(const std::vector<unsigned char> &yuyv,
        std::vector<unsigned char> *yuv420p, int width)
{
    // TODO: maybe should be using FFMPEG, like PeopleDetector

    // copy Y plane
    for (size_t i = 0; i < yuyv.size(); i += 2) {
        yuv420p->push_back(yuyv[i]);
    }

    // copy U and drop every other row
    for (size_t i = 1; i < yuyv.size(); i += 4) {
        int row = i / (width * 2);
        if (row % 2 == 1) {
            continue;
        }
        yuv420p->push_back(yuyv[i]);
    }

    // copy V and drop every other row
    for (size_t i = 3; i < yuyv.size(); i += 4) {
        int row = i / (width * 2);
        if (row % 2 == 1) {
            continue;
        }
        yuv420p->push_back(yuyv[i]);
    }
}

bool UsbCamera::getLiveImage(sensor_msgs::CompressedImage *image)
{
    switch (this->pixelformat) {
        case V4L2_PIX_FMT_H264:
            image->format = "h264";
            break;
        case V4L2_PIX_FMT_MJPEG:
            image->format = "jpeg";
            break;
        case V4L2_PIX_FMT_YUYV:
            image->format = "yuv420p";
            break;
        default:
            ROS_FATAL("Unknown pixel format");
            break;
    }
    if (this->pixelformat == V4L2_PIX_FMT_H264) {
        bool keyframe;
        usb_cam_camera_grab_h264(&(image->data), &keyframe);
        if (keyframe) {
            // XXX: pretty hacky...
            image->format = "h264_keyframe";
        }
    }
    else if (this->pixelformat == V4L2_PIX_FMT_YUYV) {
        std::vector<unsigned char> temp;
        temp.reserve(2*width*height);
        usb_cam_camera_grab_raw(&temp);
        image->data.reserve(3*width*height/2);
        convertYUYVToYUV420P(temp, &(image->data), width);
    }
    else {
        usb_cam_camera_grab_mjpeg(&(image->data));
    }
    image->header.stamp = ros::Time::now();
    return true;
}

bool UsbCamera::captureImage(sensor_msgs::CompressedImage *image)
{
    return getLiveImage(image);
}

bool tryResolution(int fd, Resolution resolution, uint32_t pixelformat)
{
    v4l2_format format;
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_G_FMT, &format) == -1) {
        ROS_FATAL("Can't read format from camera");
    }
    format.fmt.pix.width = resolution.width;
    format.fmt.pix.height = resolution.height;
    format.fmt.pix.pixelformat = pixelformat;
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

UsbCamera::UsbCamera(std::string device, int width, int height, int framerate, bool preferH264, bool preferRaw)
{
    usb_cam_camera_image_t *image;

    // Adjust height and width to something this camera supports
    int fd;
    v4l2_format format;
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    uint32_t desiredPixelFormat = V4L2_PIX_FMT_MJPEG;
    if ((fd = open(device.c_str(), O_RDONLY)) == -1){
        ROS_FATAL("Can't open %s", device.c_str());
    }

    if (preferH264 || preferRaw) {
        v4l2_fmtdesc formatDescription;
        formatDescription.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        formatDescription.index = 0;
        while (ioctl(fd, VIDIOC_ENUM_FMT, &formatDescription) != -1) {
            if (preferH264 && formatDescription.pixelformat == V4L2_PIX_FMT_H264) {
                ROS_INFO("Selecting H264 camera capture format");
                desiredPixelFormat = formatDescription.pixelformat;
                break;
            }
            if (preferRaw && formatDescription.pixelformat == V4L2_PIX_FMT_YUYV) {
                ROS_INFO("Selecting YUYV camera capture format");
                desiredPixelFormat = formatDescription.pixelformat;
                break;
            }
            formatDescription.index++;
        }
    }

    if (ioctl(fd, VIDIOC_G_FMT, &format) == -1) {
        ROS_FATAL("Can't read format from %s", device.c_str());
    }
    else {
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        format.fmt.pix.pixelformat = desiredPixelFormat;
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
        if (tryResolution(fd, resolution, desiredPixelFormat)) {
            resolutions.resolutions.push_back(resolution);
        }
    }
    close(fd);

    this->width = width;
    this->height = height;
    this->pixelformat = desiredPixelFormat;

    ROS_INFO("device: %s", device.c_str());
    image = usb_cam_camera_start(device.c_str(),
                                   IO_METHOD_MMAP,
                                   desiredPixelFormat,
                                   width,
                                   height,
                                   framerate);
    if (image == nullptr) {
        usleep(1000000);
        image = usb_cam_camera_start(device.c_str(),
                                       IO_METHOD_MMAP,
                                       desiredPixelFormat,
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
