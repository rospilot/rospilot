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

class BaseCamera
{
public:
    /**
     * Gets an image from the live preview.
     *
     * Valid values for format are "jpeg" and "h264"
     *
     * Return true on success
     */
    virtual bool getLiveImage(std::string format, sensor_msgs::CompressedImage *image) = 0;
    
    /**
     * Gets captures an image. Possibly also storing it on external media.
     *
     * Valid values for format are "jpeg" and "h264"
     *
     * Return true on success
     */
    virtual bool captureImage(std::string format, sensor_msgs::CompressedImage *image) = 0;
};
