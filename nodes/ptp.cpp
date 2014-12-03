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

#include"../src/cpp/ptp.h"

class PtpNode
{
private:
    ros::NodeHandle node;
    // NOTE: We don't need to guard this with a mutex, because callbacks
    // are called in spinOnce() in the main thread
    PtpCamera camera;
    ros::Publisher imagePub;
    ros::ServiceServer captureServiceServer;

private:
    bool sendPreview()
    {
        sensor_msgs::CompressedImage image;
        if(camera.getLiveImage("jpeg", &image)) {
            imagePub.publish(image);
            return true;
        }
        return false;
    }

public:
    PtpNode() : node("camera") 
    {
        imagePub = node.advertise<sensor_msgs::CompressedImage>(
                "image_raw/compressed", 1);
        captureServiceServer = node.advertiseService(
                "capture_image", 
                &PtpNode::captureImageCallback,
                this);
    }

    bool spin()
    {
        ROS_INFO("ptp node is running.");
                
        while (node.ok())
        {
            // Process any pending service callbacks
            ros::spinOnce();
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
        return camera.captureImage("jpeg", &response.image);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ptp");
    PtpNode a;
    a.spin();
    return 0;
}
