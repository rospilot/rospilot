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
#include <libavcodec/avcodec.h>
#include <libavutil/common.h>
}

bool SoftwareH264Encoder::encodeInPlace(sensor_msgs::CompressedImage *image,
        bool *keyFrame)
{
    if (image->format != "yuv420p") {
        ROS_ERROR("Image is not in yuv420p format");
        return false;
    }

    sourceFrame->data[0] = image->data.data();
    sourceFrame->data[1] = sourceFrame->data[0] + width * height;
    sourceFrame->data[2] = sourceFrame->data[1] + width * height / 4;
    sourceFrame->linesize[0] = width;
    sourceFrame->linesize[1] = width / 2;
    sourceFrame->linesize[2] = width / 2;
    sourceFrame->pts = (1.0 / 30) * 90 * frameCounter;
    frameCounter++;

    AVPacket packet;
    av_init_packet(&packet);
    packet.data = new uint8_t[image->data.size()];
    packet.size = image->data.size();
    int gotPacket;
    if (avcodec_encode_video2(context, &packet, sourceFrame, &gotPacket) != 0) {
        ROS_ERROR("Error during h264 encoding");
        delete[] packet.data;
        return false;
    }
    if (gotPacket != 1) {
        delete[] packet.data;
        return false;
    }

    *keyFrame = (packet.flags & AV_PKT_FLAG_KEY);

    // XXX: Do we need to output the delayed frames here? by passing null as the 
    // frame while the output buffer is getting populated

    image->data.clear();
    image->format = "h264";
    for (int i = 0; i < packet.size; i++) {
        image->data.push_back(packet.data[i]);
    }
    delete[] packet.data;

    return true;
}

AVFrame *SoftwareH264Encoder::allocFrame()
{
    AVFrame *frame;
    uint8_t *buf;
    int size;

    frame = avcodec_alloc_frame();
    size = avpicture_get_size(pixelFormat, width, height);
    buf = (uint8_t*) av_malloc(size);
    avpicture_fill((AVPicture *)frame, buf, pixelFormat, width, height);
    return frame;
}

SoftwareH264Encoder::SoftwareH264Encoder(int width, int height)
{
    avcodec_register_all();
    this->width = width;
    this->height = height;
    this->pixelFormat = PIX_FMT_YUV420P;
    
    encoder = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!encoder) {
        ROS_ERROR("Could not find H264 encoder");
    }

    context = avcodec_alloc_context3(encoder);
    avcodec_get_context_defaults3(context, encoder);
    sourceFrame = allocFrame();

    context->codec_id = encoder->id;
    context->width = width;
    context->height = height;

    context->pix_fmt = pixelFormat;
    context->codec_type = AVMEDIA_TYPE_VIDEO;

    context->bit_rate = 400000;
    context->time_base = (AVRational){1, 25};
    context->gop_size = 12;
    context->flags |= CODEC_FLAG_GLOBAL_HEADER;

    if (avcodec_open2(context, encoder, nullptr) < 0) {
        ROS_ERROR("Could not open h264 encoder");
    }
}
