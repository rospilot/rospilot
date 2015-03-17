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
#ifndef ROSPILOT_TRANSCODER_H
#define ROSPILOT_TRANSCODER_H

#include<sensor_msgs/CompressedImage.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <rospilot_deps/mfc/io_dev.h>
}

class H264Encoder
{
public:
    virtual bool encodeInPlace(sensor_msgs::CompressedImage *image, 
            bool *keyFrame) = 0;
    
    virtual ~H264Encoder() {};
};

/**
 * Designed to work with the MFC hardware module in the Odroid U3.
 */
class ExynosMultiFormatCodecH264Encoder : public H264Encoder
{
private:
    io_dev *mfc;
    // Bridge to MFC input
    io_dev *inputBridge;
    // Bridge to get output from MFC
    io_dev *outputBridge;
    io_dev *deviceChain[3];

public:
    bool encodeInPlace(sensor_msgs::CompressedImage *image,
            bool *keyFrame) override;

    ExynosMultiFormatCodecH264Encoder(int width, int height);

    ~ExynosMultiFormatCodecH264Encoder() override;
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
            bool *keyFrame) override;
    
    SoftwareH264Encoder(int width, int height);

private:
    AVFrame *allocFrame();
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
    uint8_t *outputBuffer = new uint8_t[1];
    int outputBufferSize = 1;

public:
    bool decodeInPlace(sensor_msgs::CompressedImage *image);

    JpegDecoder(int width, int height, PixelFormat outputPixelFormat);

    ~JpegDecoder();
};

#endif
