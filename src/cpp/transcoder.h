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
#include<stdio.h>
#include<iostream>
#include<gphoto2/gphoto2.h>
#include<gphoto2/gphoto2-context.h>

#include<ros/ros.h>
#include<rospilot/CaptureImage.h>
#include<std_srvs/Empty.h>
#include<sensor_msgs/fill_image.h>
#include<sensor_msgs/CompressedImage.h>

#include<base_camera.h>

extern "C" {
#include <libavutil/mathematics.h>
#include <linux/videodev2.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/common.h>
}

class H264Encoder
{
public:
    virtual bool encodeInPlace(sensor_msgs::CompressedImage *image, 
            bool *keyFrame) = 0;
    
    virtual ~H264Encoder() {};
};

class SoftwareH264Encoder : public H264Encoder
{
private:
    int width;
    int height;
    int frameCounter = 0;
    AVCodec *encoder;
    PixelFormat pixelFormat;
    AVFrame *sourceFrame;
    AVCodecContext *context;

public:

    bool encodeInPlace(sensor_msgs::CompressedImage *image,
            bool *keyFrame) override
    {
        if (image->format != "nv12") {
            ROS_ERROR("Image is not in nv12 format");
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

        // Data is going to be compressed, so this is a safe upper bound on the size
        uint8_t *outputBuffer = new uint8_t[image->data.size()];
        int outputSize = avcodec_encode_video(context, outputBuffer, image->data.size(), sourceFrame);
        
        if (context->coded_frame->key_frame) {
            *keyFrame = true;
        }

        // XXX: Do we need to output the delayed frames here? by passing null as the 
        // frame while the output buffer is getting populated

        image->data.clear();
        image->format = "h264";
        for (int i = 0; i < outputSize; i++) {
            image->data.push_back(outputBuffer[i]);
        }
        delete[] outputBuffer;
        *keyFrame = context->coded_frame->key_frame;

        return true;
    }
    
    AVFrame *allocFrame()
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

    SoftwareH264Encoder(int width, int height, PixelFormat pixelFormat)
    {
        avcodec_register_all();
        this->width = width;
        this->height = height;
        this->pixelFormat = pixelFormat;
        
        encoder = avcodec_find_encoder(CODEC_ID_H264);
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
};

class JpegDecoder
{
private:
    int width;
    int height;
    AVCodec *decoder;
    AVCodecContext *context;
    AVFrame *sourceFrame;
    AVFrame *outputFrame;
    PixelFormat outputPixelFormat;

public:
    bool decodeInPlace(sensor_msgs::CompressedImage *image)
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
        uint8_t *outputBuffer = new uint8_t[outputSize];

        int size = avpicture_layout((AVPicture *) outputFrame, outputPixelFormat, width, height,
                outputBuffer, outputSize);
        if (size != outputSize) {
            ROS_ERROR("avpicture_layout failed: %d",size);
            return false;
        }

        image->data.clear();
        image->format = "nv12";
        for (int i = 0; i < outputSize; i++) {
            image->data.push_back(outputBuffer[i]);
        }
        delete[] outputBuffer;

        return true;
    }

    JpegDecoder(int width, int height, PixelFormat outputPixelFormat)
    {
        avcodec_register_all();
        this->width = width;
        this->height = height;
        this->outputPixelFormat = outputPixelFormat;
        
        decoder = avcodec_find_decoder(CODEC_ID_MJPEG);
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
};

