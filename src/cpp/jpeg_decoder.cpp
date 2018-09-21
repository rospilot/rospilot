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
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(55,28,1)
#include <libavutil/frame.h>
#endif
#include <libavutil/imgutils.h>
}

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,28,1)
#define av_frame_alloc  avcodec_alloc_frame
#endif

namespace rospilot {

using std::vector;
using sensor_msgs::CompressedImage;

bool FFmpegJpegDecoder::decodeInPlace(sensor_msgs::CompressedImage *image)
{
    if (image->format != "jpeg") {
        ROS_ERROR("Image is not a jpeg");
        return false;
    }

    AVPacket packet;
    av_init_packet(&packet);

    packet.size = image->data.size();
    packet.data = (unsigned char*) image->data.data();
    if (avcodec_send_packet(context, &packet) != 0) {
        ROS_ERROR("Error decoding frame");
        return false;
    }
    int receiveReturnCode = avcodec_receive_frame(context, sourceFrame);
    if (receiveReturnCode != 0 && receiveReturnCode != AVERROR(EAGAIN)) {
        ROS_ERROR("Error decoding frame");
        return false;
    }
    if (receiveReturnCode == AVERROR(EAGAIN)) {
        // Frame decoded yet. Must send more packets.
        return false;
    }

    SwsContext *video_sws = sws_getContext(width, height, context->pix_fmt, width, height, 
            outputPixelFormat, SWS_BILINEAR, NULL, NULL, NULL);
    sws_scale(video_sws, sourceFrame->data, sourceFrame->linesize, 0, height,
            outputFrame->data, outputFrame->linesize );
    sws_freeContext(video_sws);  
    
    int outputSize = av_image_get_buffer_size(outputPixelFormat, width, height, 1);
    if (outputSize > outputBufferSize) {
        delete[] outputBuffer;
        outputBuffer = new uint8_t[outputSize];
        outputBufferSize = outputSize;
    }

    int size = av_image_copy_to_buffer(
            outputBuffer,
            outputSize,
            outputFrame->data,
            outputFrame->linesize,
            outputPixelFormat,
            width,
            height,
            1);
    if (size != outputSize) {
        ROS_ERROR("av_image_copy_to_buffer failed: %d",size);
        return false;
    }

    image->data.clear();
    if (outputPixelFormat == AV_PIX_FMT_YUV420P) {
        image->format = "yuv420p";
    }
    else if (outputPixelFormat == AV_PIX_FMT_YUVJ422P) {
        image->format = "yuvj422p";
    }
    else {
        ROS_ERROR("Unknown pixel format");
    }
    image->data.reserve(outputSize);
    image->data.insert(image->data.begin(), outputBuffer, outputBuffer + outputSize);

    return true;
}

FFmpegJpegDecoder::FFmpegJpegDecoder(int width, int height, AVPixelFormat outputPixelFormat)
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
    sourceFrame = av_frame_alloc();
    outputFrame = av_frame_alloc();

    outputFrame->format = outputPixelFormat;
    outputFrame->width = width;
    outputFrame->height = height;
    av_frame_get_buffer(outputFrame, 1);

    context->codec_id = decoder->id;
    context->width = width;
    context->height = height;

    context->pix_fmt = AV_PIX_FMT_YUV422P;
    context->codec_type = AVMEDIA_TYPE_VIDEO;

    if (avcodec_open2(context, decoder, nullptr) < 0) {
        ROS_ERROR("Could not open MJPEG decoder");
        return;
    }
}

FFmpegJpegDecoder::~FFmpegJpegDecoder()
{
    delete[] outputBuffer;
}

bool TurboJpegDecoder::decodeInPlace(sensor_msgs::CompressedImage *image)
{
    int sub, width, height;
    int result = tjDecompressHeader2(handle,
            image->data.data(),
            image->data.size(),
            &width,
            &height,
            &sub);
    if (result == -1) {
        ROS_ERROR("Error decompressing JPEG: %s", tjGetErrorStr());
        return false;
    }
    if (TJSAMP_420 != sub && TJSAMP_422 != sub) {
        ROS_ERROR("Unexpected jpeg encoding (%d). Expected YUV420/422", sub);
        return false;
    }
    if (this->width != width || this->height != height) {
        ROS_ERROR("Incorrect width/height. Expected %dx%d. Got %dx%d", 
                this->width, this->height, width, height);
        return false;
    }

    long size = tjBufSizeYUV(width, height, sub);
    if (outputBufferSize < size) {
        delete[] outputBuffer;
        outputBuffer = new uint8_t[size];
        outputBufferSize = size;
    }
    result = tjDecompressToYUV(handle, 
                image->data.data(), 
                image->data.size(),
                outputBuffer,
                TJFLAG_FASTUPSAMPLE | TJFLAG_FASTDCT);

    int lineWidth = width / 2;
    image->data.clear();
    image->data.reserve(tjBufSizeYUV(width, height, TJSAMP_420));
    if (TJSAMP_422 == sub) {
        image->data.insert(image->data.begin(), outputBuffer, outputBuffer + height*width + lineWidth);
        // TODO: Do a proper conversion, instead of just dropping lines
        unsigned char *base = outputBuffer + height * width;
        // Drop half the lines of UV components
        for (int i = 1; i < height; i++) {
            image->data.insert(image->data.begin() + width * height + i * lineWidth,
                    base + 2 * i * lineWidth, base + 2 * i * lineWidth + lineWidth);
        }
    }
    else {
        image->data.insert(image->data.begin(), outputBuffer, outputBuffer + outputBufferSize);
    }

    if (result == -1) {
        ROS_ERROR("Error decompressing JPEG: %s", tjGetErrorStr());
        return false;
    }
    image->format = "yuv420p";
    return true;   
}

TurboJpegDecoder::TurboJpegDecoder(int width, int height, AVPixelFormat outputPixelFormat)
{
    handle = tjInitDecompress();
    this->width = width;
    this->height = height;
}

TurboJpegDecoder::~TurboJpegDecoder()
{
    tjDestroy(handle);
    delete[] outputBuffer;
}

}
