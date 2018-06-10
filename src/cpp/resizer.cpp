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
#include "resizer.h"

#include<ros/ros.h>
#include<sensor_msgs/CompressedImage.h>

extern "C" {
#include <libswscale/swscale.h>
#include <libavutil/mem.h>
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55,28,1)
#include <libavutil/frame.h>
#endif
#include <libavutil/imgutils.h>
}

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
#define av_frame_alloc  avcodec_alloc_frame
#define av_frame_free(x)   av_free(*x)
#endif

namespace rospilot {

using std::vector;
using sensor_msgs::CompressedImage;

bool Resizer::resizeInPlace(sensor_msgs::CompressedImage *image)
{
    if (originalWidth == targetWidth && originalHeight == targetHeight) {
        return true;
    }

    av_image_fill_arrays(
            sourceFrame->data,
            sourceFrame->linesize,
            image->data.data(),
            pixelFormat,
            originalWidth,
            originalHeight,
            1);

    sws_scale(
            context,
            sourceFrame->data,
            sourceFrame->linesize,
            0,
            originalHeight, 
            outputFrame->data, 
            outputFrame->linesize);


    int size = av_image_copy_to_buffer(
            outputBuffer,
            outputBufferSize,
            outputFrame->data,
            outputFrame->linesize,
            pixelFormat,
            targetWidth,
            targetHeight,
            1);
    if (size != outputBufferSize) {
        ROS_ERROR("av_image_copy_to_buffer failed: %d",size);
        return false;
    }

    image->data.clear();
    image->data.reserve(outputBufferSize);
    image->data.insert(image->data.begin(), outputBuffer, outputBuffer + outputBufferSize);

    return true;
}

Resizer::Resizer(
        int originalWidth,
        int originalHeight,
        int targetWidth,
        int targetHeight,
        AVPixelFormat pixelFormat)
{
    this->pixelFormat = pixelFormat;
    this->originalWidth = originalWidth;
    this->originalHeight = originalHeight;
    this->targetWidth = targetWidth;
    this->targetHeight = targetHeight;
    
    outputBufferSize = av_image_get_buffer_size(pixelFormat, targetWidth, targetHeight, 1);
    outputBuffer = new uint8_t[outputBufferSize];

    outputFrame = av_frame_alloc();
    outputFrame->format = pixelFormat;
    outputFrame->width = targetWidth;
    outputFrame->height = targetHeight;
    av_frame_get_buffer(outputFrame, 1);
    
    sourceFrame = av_frame_alloc();
    sourceFrame->width = originalWidth;
    sourceFrame->height = originalHeight;
    sourceFrame->format = pixelFormat;

    context = sws_getContext(
            originalWidth,
            originalHeight,
            pixelFormat, 
            targetWidth, 
            targetHeight,
            pixelFormat,
            SWS_POINT,
            nullptr,
            nullptr, 
            nullptr);
}

Resizer::~Resizer()
{
    av_frame_free(&sourceFrame);
    av_frame_free(&outputFrame);
    sws_freeContext(context);  
    delete[] outputBuffer;
}

}
