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
#include<time.h>

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

class SoftwareVideoRecorder
{
private:
    static const int FPS = 60;
    // NOTE: We don't need to guard this with a mutex, because callbacks
    // are called in spinOnce() in the main thread
    bool recording = false;
    AVFormatContext *formatContext;
    AVStream *videoStream;
    double firstFrameTime;
    int lastPTS = 0;
    bool foundKeyframe = false;
    std::string tempFilename;
    std::string filename;
    int width;
    int height;
    PixelFormat pixelFormat;

public:
    SoftwareVideoRecorder(int width, int height, PixelFormat pixelFormat)
    {
        av_register_all();
        this->width = width;
        this->height = height;
        this->pixelFormat = pixelFormat;
        if (CLOCKS_PER_SEC < 1000) {
            ROS_ERROR("Clock is not precise enough to record video");
            exit(1);
        }
    }
    
    void writeFrame(sensor_msgs::CompressedImage *image, bool keyFrame)
    {
        if (!recording) {
            return;
        }
        double currentTime = clock() / ((double) CLOCKS_PER_SEC);
        if (!foundKeyframe && keyFrame) {
            foundKeyframe = true;
            firstFrameTime = currentTime;
            lastPTS = -1;
        }
        
        if (!foundKeyframe) {
            return;
        }

        int pts = (int) ((currentTime - firstFrameTime) * FPS);
        ROS_INFO("pts: %d", pts);
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

    AVStream *createVideoStream(AVFormatContext *oc, enum CodecID codec_id)
    {
        AVCodecContext *c;
        AVStream *stream;
        AVCodec *codec;

        codec = avcodec_find_encoder(CODEC_ID_H264);
        if (!codec) {
            ROS_ERROR("codec not found");
        }

        stream = avformat_new_stream(oc, codec);
        if (!stream) {
            ROS_ERROR("Could not alloc stream");
        }

        c = stream->codec;
        avcodec_get_context_defaults3(c, codec);

        c->codec_id = codec_id;
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
        formatContext->oformat = av_guess_format("mp4", NULL, NULL);

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
        videoStream = createVideoStream(formatContext, 
                formatContext->oformat->video_codec);
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
        ROS_INFO("RECORDING output as %s with vcodec %d, short name = %s", 
                formatContext->oformat->long_name,
                formatContext->oformat->video_codec,
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
        rename(tempFilename.c_str(), filename.c_str());
        return true;
    }

    ~SoftwareVideoRecorder()
    {
    }
};

#endif
