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
#ifndef ROSPILOT_VIDEO_RECORDER_H
#define ROSPILOT_VIDEO_RECORDER_H

#include<stdio.h>
#include<iostream>
#include<chrono>
#include<errno.h>

#include<ros/ros.h>
#include<rospilot/CaptureImage.h>
#include<std_srvs/Empty.h>
#include<sensor_msgs/fill_image.h>
#include<sensor_msgs/CompressedImage.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avassert.h>
#include <libavutil/channel_layout.h>
#include <libavutil/opt.h>
#include <libavutil/mathematics.h>
#include <libswscale/swscale.h>
#include <libavutil/common.h>
#include <libavformat/avformat.h>
}

using namespace std::chrono;

class SoftwareVideoRecorder
{
private:
    static const int FPS = 60;
    // NOTE: We don't need to guard this with a mutex, because callbacks
    // are called in spinOnce() in the main thread
    bool recording = false;
    AVFormatContext *formatContext;
    AVStream *videoStream;
    time_point<high_resolution_clock> firstFrameTime;
    int lastPTS = 0;
    bool foundKeyframe = false;
    std::string tempFilename;
    std::string filename;
    int width;
    int height;
    PixelFormat pixelFormat;
    CodecID codecId;

public:
    SoftwareVideoRecorder(int width, int height, PixelFormat pixelFormat, CodecID codecId)
    {
        av_register_all();
        this->width = width;
        this->height = height;
        this->pixelFormat = pixelFormat;
        this->codecId = codecId;
    }
    
    void writeFrame(sensor_msgs::CompressedImage *image, bool keyFrame)
    {
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

            pkt.pts = pts;
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

    AVStream *createVideoStream(AVFormatContext *oc)
    {
        AVCodecContext *c;
        AVStream *stream;
        AVCodec *codec;

        codec = avcodec_find_encoder(codecId);
        if (!codec) {
            ROS_ERROR("codec not found");
        }

        stream = avformat_new_stream(oc, codec);
        if (!stream) {
            ROS_ERROR("Could not alloc stream");
        }

        c = stream->codec;
        avcodec_get_context_defaults3(c, codec);

        c->codec_id = codecId;
        c->bit_rate = 400000;
        c->width = this->width;
        c->height = this->height;
        c->time_base.den = FPS;
        c->time_base.num = 1;
        c->gop_size = 12;
        c->pix_fmt = this->pixelFormat;
        c->flags |= CODEC_FLAG_GLOBAL_HEADER;

        return stream;
    }
    
    bool start(const char *name)
    {
        filename = std::string(name);
        char *tempname = tempnam(nullptr, nullptr);
        tempFilename = std::string(tempname);
        free(tempname);
        formatContext = avformat_alloc_context();
        formatContext->oformat = av_guess_format(nullptr, name, nullptr);

        formatContext->priv_data =
            av_mallocz(formatContext->oformat->priv_data_size);
        if (formatContext->oformat->priv_class) {
            *(const AVClass**)formatContext->priv_data =
                formatContext->oformat->priv_class;
            av_opt_set_defaults(formatContext->priv_data);
        }

        strncpy(formatContext->filename, 
                tempFilename.c_str(),
                sizeof(formatContext->filename));
        // Check that this is a valid combination
        if(avformat_query_codec(formatContext->oformat, codecId, FF_COMPLIANCE_NORMAL) != 1) {
            ROS_FATAL("Codec %d not compatible with output format %s", codecId, 
                    formatContext->oformat->long_name);
        }
        videoStream = createVideoStream(formatContext);
        if (avcodec_open2(videoStream->codec, nullptr, nullptr) < 0) {
            ROS_ERROR("could not open codec");
        }
        av_dump_format(formatContext, 0, tempFilename.c_str(), 1);

        if (avio_open(&formatContext->pb, 
                    tempFilename.c_str(),
                    AVIO_FLAG_WRITE) < 0) {
            fprintf(stderr, "Could not open '%s'", tempFilename.c_str());
            return 1;
        }

        avformat_write_header(formatContext, nullptr);
        foundKeyframe = false;
        recording = true;
        ROS_INFO("Start recording output as %s with vcodec %d, short name = %s", 
                formatContext->oformat->long_name,
                codecId,
                formatContext->oformat->name);
        return true;
    }
    
    bool stop()
    {
        recording = false;
        av_write_trailer(formatContext);
        avcodec_close(videoStream->codec);
        for (int i = 0; i < formatContext->nb_streams; i++) {
            av_freep(&formatContext->streams[i]->codec);
            av_freep(&formatContext->streams[i]);
        }
        avio_close(formatContext->pb);
        av_free(formatContext);
        ROS_INFO("Finializing video file: %s", filename.c_str());
        if(rename(tempFilename.c_str(), filename.c_str()) != 0) {
            ROS_INFO("Error moving temp file: %s", strerror(errno));
        }
        else {
            ROS_INFO("Finished recording video");
        }
        return true;
    }

    ~SoftwareVideoRecorder()
    {
    }
};

#endif
