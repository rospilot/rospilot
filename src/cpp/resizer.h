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
#ifndef ROSPILOT_RESIZER_H
#define ROSPILOT_RESIZER_H

#include<sensor_msgs/CompressedImage.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55,28,1)
#include <libavutil/frame.h>
#endif
}

namespace rospilot {

class Resizer 
{
private:
    SwsContext *context;
    AVFrame *sourceFrame;
    AVFrame *outputFrame;
    uint8_t *outputBuffer;
    int outputBufferSize;
    PixelFormat pixelFormat;
    int originalWidth, originalHeight, targetWidth, targetHeight;

public:
    bool resizeInPlace(sensor_msgs::CompressedImage *image);

    Resizer(
        int originalWidth,
        int originalHeight,
        int targetWidth,
        int targetHeight,
        PixelFormat pixelFormat);

    ~Resizer();
};

}

#endif

