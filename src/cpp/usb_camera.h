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

#include<sensor_msgs/CompressedImage.h>
#include<base_camera.h>

class UsbCamera : public BaseCamera
{
private:
    int width;
    int height;

public:
    int getWidth();

    int getHeight();

    bool getLiveImage(sensor_msgs::CompressedImage *image) override;
    
    bool captureImage(sensor_msgs::CompressedImage *image) override;

    UsbCamera(std::string device, int width, int height, int framerate);

    ~UsbCamera();
};

#endif
