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
#include <libavutil/opt.h>
#include <libavutil/common.h>
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55,28,1)
#include <libavutil/frame.h>
#endif
}

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
#define av_frame_alloc  avcodec_alloc_frame
#endif

namespace rospilot {

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

    frame = av_frame_alloc();
    size = avpicture_get_size(pixelFormat, width, height);
    buf = (uint8_t*) av_malloc(size);
    avpicture_fill((AVPicture *)frame, buf, pixelFormat, width, height);
    return frame;
}

SoftwareH264Encoder::SoftwareH264Encoder(H264Settings settings)
{
    avcodec_register_all();
    this->width = settings.width;
    this->height = settings.height;
    this->pixelFormat = PIX_FMT_YUV420P;
    
    encoder = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!encoder) {
        ROS_ERROR("Could not find H264 encoder");
    }

    context = avcodec_alloc_context3(encoder);
    avcodec_get_context_defaults3(context, encoder);
    sourceFrame = allocFrame();

    context->codec_id = encoder->id;
    context->width = settings.width;
    context->height = settings.height;

    context->pix_fmt = pixelFormat;
    context->codec_type = AVMEDIA_TYPE_VIDEO;

    context->bit_rate = settings.bit_rate;
    context->time_base = (AVRational){1, 25};
    context->gop_size = settings.gop_size;
    context->level = settings.level;

    // Not sure this does anything, so set the "profile" on priv_data also
    if (settings.profile == CONSTRAINED_BASELINE) {
        context->profile = FF_PROFILE_H264_CONSTRAINED_BASELINE;
        av_opt_set(context->priv_data, "profile", "baseline", AV_OPT_SEARCH_CHILDREN);
    }
    else if (settings.profile == HIGH) {
        context->profile = FF_PROFILE_H264_HIGH;
        av_opt_set(context->priv_data, "profile", "high", AV_OPT_SEARCH_CHILDREN);
    }
    else {
        ROS_ERROR("Unknown H264 profile");
    }
    if (settings.zero_latency) {
        av_opt_set(context->priv_data, "tune", "zerolatency", AV_OPT_SEARCH_CHILDREN);
        av_opt_set(context->priv_data, "preset", "ultrafast", AV_OPT_SEARCH_CHILDREN);
    }

    if (avcodec_open2(context, encoder, nullptr) < 0) {
        ROS_ERROR("Could not open h264 encoder");
    }
}

}
