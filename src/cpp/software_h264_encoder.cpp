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
#include "h264_utils.h"

#include<ros/ros.h>
#include<sensor_msgs/CompressedImage.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/common.h>
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55,28,1)
#include <libavutil/frame.h>
#endif
#include <libavutil/imgutils.h>
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
    av_new_packet(&packet, image->data.size());
    if (avcodec_send_frame(context, sourceFrame) != 0) {
        ROS_ERROR("Error during h264 encoding");
        av_packet_unref(&packet);
        return false;
    }
    int receiveReturnCode = avcodec_receive_packet(context, &packet);
    if (receiveReturnCode != 0 && receiveReturnCode != AVERROR(EAGAIN)) {
        ROS_ERROR("Error during h264 encoding");
        av_packet_unref(&packet);
        return false;
    }
    if (receiveReturnCode == AVERROR(EAGAIN)) {
        // No packet received. Must send more frames.
        av_packet_unref(&packet);
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
    av_packet_unref(&packet);

    return true;
}

AVFrame *SoftwareH264Encoder::allocFrame()
{
    AVFrame *frame;
    uint8_t *buf;
    int size;

    frame = av_frame_alloc();
    size = av_image_get_buffer_size(pixelFormat, width, height, 1);
    buf = (uint8_t*) av_malloc(size);
    av_image_fill_arrays(frame->data, frame->linesize, buf, pixelFormat, width, height, 1);
    return frame;
}

std::vector<uint8_t> SoftwareH264Encoder::getSPS()
{
    std::vector<uint8_t> temp;
    for (int i = 0; i < context->extradata_size; i++) {
        temp.push_back(context->extradata[i]);
    }
    std::vector<uint8_t> sps;
    std::vector<uint8_t> pps;
    tryExtractSPSandPPS(temp, sps, pps);
    return sps;
}

std::vector<uint8_t> SoftwareH264Encoder::getPPS()
{
    std::vector<uint8_t> temp;
    for (int i = 0; i < context->extradata_size; i++) {
        temp.push_back(context->extradata[i]);
    }
    std::vector<uint8_t> sps;
    std::vector<uint8_t> pps;
    tryExtractSPSandPPS(temp, sps, pps);
    return pps;
}

SoftwareH264Encoder::SoftwareH264Encoder(H264Settings settings)
{
    avcodec_register_all();
    this->width = settings.width;
    this->height = settings.height;
    this->pixelFormat = AV_PIX_FMT_YUV420P;
    
    encoder = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!encoder) {
        ROS_ERROR("Could not find H264 encoder");
    }

    context = avcodec_alloc_context3(encoder);
    avcodec_get_context_defaults3(context, encoder);
    sourceFrame = allocFrame();
    sourceFrame->width = this->width;
    sourceFrame->height = this->height;
    sourceFrame->format = this->pixelFormat;

    context->codec_id = encoder->id;
    context->width = settings.width;
    context->height = settings.height;

    context->pix_fmt = pixelFormat;
    context->codec_type = AVMEDIA_TYPE_VIDEO;

    context->bit_rate = settings.bit_rate;
    context->time_base = (AVRational){1, 25};
    context->gop_size = settings.gop_size;
    context->level = settings.level;
    context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

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
