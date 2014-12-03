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
#include<stdio.h>
#include<iostream>
#include<gphoto2/gphoto2.h>
#include<gphoto2/gphoto2-context.h>

#include<ros/ros.h>
#include<rospilot/CaptureImage.h>
#include<std_srvs/Empty.h>
#include<sensor_msgs/fill_image.h>
#include<sensor_msgs/CompressedImage.h>

#include<ptp.h>
#include<usb_camera.h>

class CameraNode
{
private:
    ros::NodeHandle node;
    // NOTE: We don't need to guard this with a mutex, because callbacks
    // are called in spinOnce() in the main thread
    BaseCamera *camera;
    ros::Publisher imagePub;
    ros::ServiceServer captureServiceServer;

    std::string videoDevice;
    std::string pixelFormat; // "jpeg" or "h264"
    int width;
    int height;
    int framerate;

private:
    bool sendPreview()
    {
        sensor_msgs::CompressedImage image;
        if(camera != nullptr && camera->getLiveImage(&image)) {
            imagePub.publish(image);
            return true;
        }
        return false;
    }

    BaseCamera *createCamera()
    {
        std::string cameraType;
        node.param("camera_type", cameraType, std::string("usb"));

        node.param("video_device", videoDevice, std::string("/dev/video0"));
        node.param("image_width", width, 640);
        node.param("image_height", height, 480);
        node.param("framerate", framerate, 30);
        node.param("pixel_format", pixelFormat, std::string("jpeg"));

        if (cameraType == "ptp") {
            return new PtpCamera();
        }
        else if (cameraType == "usb") {
            return new UsbCamera(videoDevice, width, height, framerate);
        }
        else {
            ROS_FATAL("Unsupported camera type: %s", cameraType.c_str());
            return nullptr;
        }
    }

public:
    CameraNode() : node("~") 
    {
        imagePub = node.advertise<sensor_msgs::CompressedImage>(
                "image_raw/compressed", 1);
        captureServiceServer = node.advertiseService(
                "capture_image", 
                &CameraNode::captureImageCallback,
                this);
        camera = createCamera();
    }

    ~CameraNode()
    {
        if (camera != nullptr) {
            delete camera;
        }
    }

    bool spin()
    {
        ROS_INFO("camera node is running.");
                
        while (node.ok())
        {
            // Process any pending service callbacks
            ros::spinOnce();
            std::string newVideoDevice;
            node.getParam("video_device", newVideoDevice);
            if (newVideoDevice != videoDevice) {
                if (camera != nullptr) {
                    delete camera;
                }
                camera = createCamera();
            }
            if(!sendPreview()) {
                // Sleep and hope the camera recovers
                usleep(1000*1000);
            }
            // Run at 1kHz
            usleep(1000);
        }
        return true;
    }

    bool captureImageCallback(rospilot::CaptureImage::Request& request, 
            rospilot::CaptureImage::Response &response)
    {
        if (camera != nullptr) {
            return camera->captureImage(&response.image);
        }
        return false;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera");
    CameraNode a;
    a.spin();
    return 0;
}
