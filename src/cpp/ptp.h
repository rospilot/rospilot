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
#ifndef ROSPILOT_PTP_H
#define ROSPILOT_PTP_H

#include<gphoto2/gphoto2.h>
#include<gphoto2/gphoto2-context.h>

#include<sensor_msgs/CompressedImage.h>

#include<base_camera.h>

class PtpCamera : public BaseCamera
{
private:
    GPContext *context;
    CameraFile *cameraFile;
    Camera *camera;

public:
    bool getLiveImage(sensor_msgs::CompressedImage *image) override;
    
    bool captureImage(sensor_msgs::CompressedImage *image) override;

private:
    void checkErrorCode(int errorCode, std::string message);

    bool makeImage(CameraFile *file, sensor_msgs::CompressedImage *image);

public:
    PtpCamera();

    ~PtpCamera();
};

#endif
