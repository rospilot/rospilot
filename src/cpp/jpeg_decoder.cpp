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
#include "transcoders.h"

#include<ros/ros.h>
#include<sensor_msgs/CompressedImage.h>

extern "C" {
#include <libswscale/swscale.h>
}

using std::vector;
using sensor_msgs::CompressedImage;

bool JpegDecoder::decodeInPlace(sensor_msgs::CompressedImage *image)
{
    if (image->format != "jpeg") {
        ROS_ERROR("Image is not a jpeg");
        return false;
    }

    AVPacket packet;
    av_init_packet(&packet);

    packet.size = image->data.size();
    packet.data = (unsigned char*) image->data.data();
    int gotPicture;
    int decodedLength = avcodec_decode_video2(context, sourceFrame, &gotPicture, &packet);

    if (decodedLength <= 0) {
        ROS_ERROR("Error decoding frame: %d", decodedLength);
        return false;
    }

    SwsContext *video_sws = sws_getContext(width, height, context->pix_fmt, width, height, 
            outputPixelFormat, SWS_BILINEAR, NULL, NULL, NULL);
    sws_scale(video_sws, sourceFrame->data, sourceFrame->linesize, 0, height,
            outputFrame->data, outputFrame->linesize );
    sws_freeContext(video_sws);  
    
    int outputSize = avpicture_get_size(outputPixelFormat, width, height);
    if (outputSize > outputBufferSize) {
        delete[] outputBuffer;
        outputBuffer = new uint8_t[outputSize];
        outputBufferSize = outputSize;
    }

    int size = avpicture_layout((AVPicture *) outputFrame, outputPixelFormat, width, height,
            outputBuffer, outputSize);
    if (size != outputSize) {
        ROS_ERROR("avpicture_layout failed: %d",size);
        return false;
    }

    image->data.clear();
    if (outputPixelFormat == PIX_FMT_YUV420P) {
        image->format = "yuv420p";
    }
    else if (outputPixelFormat == PIX_FMT_YUVJ422P) {
        image->format = "yuvj422p";
    }
    else {
        ROS_ERROR("Unknown pixel format");
    }
    image->data.reserve(outputSize);
    image->data.insert(image->data.begin(), outputBuffer, outputBuffer + outputSize);

    return true;
}

JpegDecoder::JpegDecoder(int width, int height, PixelFormat outputPixelFormat)
{
    avcodec_register_all();
    this->width = width;
    this->height = height;
    this->outputPixelFormat = outputPixelFormat;
    
    decoder = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if (!decoder)
    {
        ROS_ERROR("Could not find MJPEG decoder");
        return;
    }

    context = avcodec_alloc_context3(decoder);
    if (!context)
    {
        ROS_ERROR("Could not create MJPEG decoder context");
        return;
    }
    sourceFrame = avcodec_alloc_frame();
    outputFrame = avcodec_alloc_frame();

    avpicture_alloc((AVPicture *)outputFrame, outputPixelFormat, width, height);

    context->codec_id = decoder->id;
    context->width = width;
    context->height = height;

    context->pix_fmt = PIX_FMT_YUV422P;
    context->codec_type = AVMEDIA_TYPE_VIDEO;

    if (avcodec_open2(context, decoder, nullptr) < 0) {
        ROS_ERROR("Could not open MJPEG decoder");
        return;
    }
}

JpegDecoder::~JpegDecoder()
{
    delete[] outputBuffer;
}
