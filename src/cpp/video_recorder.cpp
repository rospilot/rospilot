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

#include "video_recorder.h"

#include<stdio.h>
#include<iostream>
#include<chrono>
#include<errno.h>

#include<ros/ros.h>
#include<sensor_msgs/CompressedImage.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/mathematics.h>
#include <libavformat/avformat.h>
}

namespace rospilot {

using namespace std::chrono;

SoftwareVideoRecorder::SoftwareVideoRecorder(AVPixelFormat pixelFormat, H264Settings settings, std::string mediaPath)
{
    av_register_all();
    this->width = settings.width;
    this->height = settings.height;
    this->pixelFormat = pixelFormat;
    this->settings = settings;
    this->tempFilename = mediaPath + "/.tmp.mp4";
}

void SoftwareVideoRecorder::addFrame(sensor_msgs::CompressedImage *image, bool keyFrame)
{
    // acquire lock so that we can read this->recording
    std::lock_guard<std::mutex> guard(lock);
    if (!recording) {
        return;
    }

    time_point<high_resolution_clock> currentTime = high_resolution_clock::now();
    if (!foundKeyframe && keyFrame) {
        foundKeyframe = true;
        firstFrameTime = currentTime;
        lastPTS = -1;
    }
    
    if (!foundKeyframe) {
        return;
    }

    duration<double> duration = (currentTime - firstFrameTime);
    int pts = (int) (duration.count() * FPS);
    if (pts == lastPTS) {
        // If we're receiving frames faster than 60 fps, just drop them.
        return;
    }
    lastPTS = pts;

    // Skip empty frames
    if (image->data.size() > 0) {
        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.dts = av_rescale_q(pts, (AVRational){1, FPS},
                formatContext->streams[0]->time_base);
        pkt.pts = pkt.dts;
        if (keyFrame) {
            pkt.flags |= AV_PKT_FLAG_KEY;
        }

        pkt.stream_index = 0;
        pkt.data = image->data.data();
        pkt.size = image->data.size();

        int ret = av_interleaved_write_frame(formatContext, &pkt);
        if (ret != 0) {
            ROS_ERROR("Error while writing video frame: %d", ret);
        }
    }
}

AVStream *SoftwareVideoRecorder::createVideoStream(AVFormatContext *oc)
{
    AVStream *stream;
    AVCodec *codec;

    codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        ROS_ERROR("codec not found");
    }

    stream = avformat_new_stream(oc, codec);
    if (!stream) {
        ROS_ERROR("Could not alloc stream");
    }

    codecContext = avcodec_alloc_context3(codec);

    stream->time_base.den = FPS;
    stream->time_base.num = 1;
    codecContext->time_base = stream->time_base;

    codecContext->bit_rate = this->settings.bit_rate;
    codecContext->width = this->width;
    codecContext->height = this->height;
    codecContext->gop_size = this->settings.gop_size;
    // Not sure this does anything, so set the "profile" on priv_data also
    if (settings.profile == CONSTRAINED_BASELINE) {
        codecContext->profile = FF_PROFILE_H264_CONSTRAINED_BASELINE;
        av_opt_set(codecContext->priv_data, "profile", "baseline", AV_OPT_SEARCH_CHILDREN);
    }
    else if (settings.profile == HIGH) {
        codecContext->profile = FF_PROFILE_H264_HIGH;
        av_opt_set(codecContext->priv_data, "profile", "high", AV_OPT_SEARCH_CHILDREN);
    }
    else {
        ROS_ERROR("Unknown H264 profile");
    }
    if (settings.zero_latency) {
        av_opt_set(codecContext->priv_data, "tune", "zerolatency", AV_OPT_SEARCH_CHILDREN);
        av_opt_set(codecContext->priv_data, "preset", "ultrafast", AV_OPT_SEARCH_CHILDREN);
    }
    codecContext->level = this->settings.level;
    codecContext->pix_fmt = this->pixelFormat;
    codecContext->flags |= CODEC_FLAG_GLOBAL_HEADER;

    avcodec_parameters_from_context(stream->codecpar, codecContext);

    if (avcodec_open2(codecContext, codec, nullptr) < 0) {
        ROS_ERROR("could not open codec");
    }

    return stream;
}

bool SoftwareVideoRecorder::start(const char *name)
{
    AVCodecID codecId = AV_CODEC_ID_H264;
    std::lock_guard<std::mutex> guard(lock);
    filename = std::string(name);
    formatContext = avformat_alloc_context();
    formatContext->oformat = av_guess_format(nullptr, name, nullptr);

    formatContext->priv_data =
        av_mallocz(formatContext->oformat->priv_data_size);
    if (formatContext->oformat->priv_class) {
        *(const AVClass**)formatContext->priv_data =
            formatContext->oformat->priv_class;
        av_opt_set_defaults(formatContext->priv_data);
    }

    // Clear the file before we start writing
    remove(tempFilename.c_str());

    strncpy(formatContext->filename, 
            tempFilename.c_str(),
            sizeof(formatContext->filename));
    // Check that this is a valid combination
    if(avformat_query_codec(formatContext->oformat, codecId, FF_COMPLIANCE_NORMAL) != 1) {
        ROS_FATAL("Codec %d not compatible with output format %s", codecId, 
                formatContext->oformat->long_name);
    }
    videoStream = createVideoStream(formatContext);
    av_dump_format(formatContext, 0, tempFilename.c_str(), 1);

    if (avio_open(&formatContext->pb, 
                tempFilename.c_str(),
                AVIO_FLAG_WRITE) < 0) {
        ROS_ERROR("Could not open '%s'", tempFilename.c_str());
        return false;
    }

    if(avformat_write_header(formatContext, nullptr) < 0) {
        ROS_FATAL("Failed to write video output header");
        return false;
    }
    foundKeyframe = false;
    recording = true;
    ROS_INFO("Start recording output to %s as %s with vcodec %d, short name = %s", 
            tempFilename.c_str(),
            formatContext->oformat->long_name,
            codecId,
            formatContext->oformat->name);
    return true;
}

bool SoftwareVideoRecorder::stop()
{
    std::lock_guard<std::mutex> guard(lock);
    recording = false;
    av_write_trailer(formatContext);
    AVIOContext *pb = formatContext->pb;
    avcodec_free_context(&codecContext);
    avformat_free_context(formatContext);
    avio_close(pb);
    ROS_INFO("Finializing video file: %s", filename.c_str());
    if(rename(tempFilename.c_str(), filename.c_str()) != 0) {
        ROS_ERROR("Error moving temp file: %s", strerror(errno));
    }
    else {
        ROS_INFO("Finished recording video");
    }
    return true;
}

SoftwareVideoRecorder::~SoftwareVideoRecorder()
{
}

}
