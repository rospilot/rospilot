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
#include<vector>
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
#include <rospilot_deps/mfc/mfc.h>
#include <rospilot_deps/mfc/io_dev.h>
#include <rospilot_deps/mfc/func_dev.h>
#include <rospilot_deps/mfc/v4l_dev.h>
}

using std::vector;
using sensor_msgs::CompressedImage;

class H264Encoder
{
public:
    virtual bool encodeInPlace(sensor_msgs::CompressedImage *image, 
            bool *keyFrame) = 0;
    
    virtual ~H264Encoder() {};
};

struct OutputBridgePriv
{
    vector<unsigned char> *image;
    bool readSuccessful;
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
            bool *keyFrame) override
    {
        if (image->format != "yuv420p") {
            ROS_ERROR("Image is not in yuv420p format");
            return false;
        }

        inputBridge->priv = &image->data;
        OutputBridgePriv *outPriv = (OutputBridgePriv *)outputBridge->priv;
        outPriv->image = &image->data;
        outPriv->readSuccessful = false;

        // XXX: There's a one frame delay in this processing pipeline,
        // so the flags for the frame we're about to get from the MFC
        // are already in the MFC's private data section. This whole thing
        // really needs to be refactored at some point.
        int flags = ((mfc_priv *) this->mfc->priv)->last_frame_flags;
        *keyFrame = (flags & V4L2_BUF_FLAG_KEYFRAME);

        if (wait_for_ready_devs(this->deviceChain, 3) < 0) {
            ROS_ERROR("Error waiting for MFC to be ready");
            return false;
        }

        if (process_pair(this->inputBridge, this->mfc) != 0) {
            ROS_ERROR("Sending buffer to MFC failed");
            return false;
        }
        
        if (process_pair(this->mfc, this->outputBridge) != 0) {
            ROS_ERROR("Reading buffer from MFC failed");
            return false;
        }

        if (outPriv->readSuccessful) {
            image->format = "h264";
        }

        return outPriv->readSuccessful;
    }

    static int copyToMFCBuffer(io_dev *dev, int nbufs, char **bufs, int *lens)
    {
        vector<unsigned char> *image = (vector<unsigned char> *) dev->priv;
        if (nbufs != 2) {
            ROS_ERROR("Expected 2 buffers and NV12 format to write to, got %d buffers", nbufs);
            return -1;
        }
        if (image->size() != lens[0] + lens[1]) {
            ROS_ERROR("Wrong image size. Trying to copy %d into %d + %d", 
                    (int) image->size(), lens[0], lens[1]);
            return -1;
        }
        // Copy Y plane
        for (int i = 0; i < lens[0]; i++) {
            bufs[0][i] = (*image)[i];
        }
        // Copy U and V pixels and interleave them
        int numPixels = lens[0];
        for (int i = 0; i < numPixels / 4; i++) {
            bufs[1][i * 2] = (*image)[numPixels + i];
            bufs[1][i * 2 + 1] = (*image)[(numPixels * 5 / 4) + i];
        }

        return 0;
    }

    static int copyFromMFCBuffer(io_dev *dev, int nbufs, char **bufs, int *lens)
    {
        vector<unsigned char> *image = ((OutputBridgePriv *) dev->priv)->image;
        ((OutputBridgePriv *) dev->priv)->readSuccessful = true;
        if (nbufs != 1) {
            ROS_ERROR("Expected 1 H264 encoded buffer. Got %d", nbufs);
            return -1;
        }

        image->clear();
        for (int i = 0; i < lens[0]; i++) {
            image->push_back(bufs[0][i]);
        }
        
        return 0;
    }

    ExynosMultiFormatCodecH264Encoder(int width, int height)
    {
        this->mfc = mfc_create("/dev/video9");
        if (mfc_set_fmt(this->mfc, DIR_IN, width, height)) {
            ROS_FATAL("Failed to set format on MFC");
        }

        if (mfc_set_codec(this->mfc, DIR_OUT, V4L2_PIX_FMT_H264)) {
            ROS_FATAL("Failed to set codec on MFC");
        }
        
        if (mfc_set_rate(this->mfc, 30)) {
            ROS_FATAL("Failed to set rate on MFC");
        }
		
        mfc_set_mpeg_control(this->mfc, V4L2_CID_MPEG_VIDEO_GOP_SIZE, 12);
        mfc_set_mpeg_control(this->mfc, V4L2_CID_MPEG_VIDEO_H264_I_PERIOD, 12);
        //mfc_set_mpeg_control(this->mfc, V4L2_CID_MPEG_VIDEO_B_FRAMES, 2);
        mfc_set_mpeg_control(this->mfc, V4L2_CID_MPEG_VIDEO_BITRATE, 400000);
        mfc_set_mpeg_control(this->mfc, V4L2_CID_MPEG_VIDEO_HEADER_MODE, 
                V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME);
        mfc_set_mpeg_control(this->mfc, V4L2_CID_MPEG_VIDEO_H264_PROFILE,
                V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);

        this->inputBridge = new io_dev();
        this->inputBridge->fd = -1;
        this->inputBridge->io[DIR_IN].type = IO_NONE;
        this->inputBridge->io[DIR_OUT].type = IO_FUNC;
        this->inputBridge->ops = new io_dev_ops();
        this->inputBridge->ops->read = &copyToMFCBuffer;
        this->inputBridge->ops->req_bufs = func_req_bufs;
        this->inputBridge->ops->enq_buf = func_enq_buf;
        this->inputBridge->ops->deq_buf = func_deq_buf;
        
        this->outputBridge = new io_dev();
        this->outputBridge->priv = new OutputBridgePriv();
        this->outputBridge->fd = -2;
        this->outputBridge->io[DIR_IN].type = IO_FUNC;
        this->outputBridge->io[DIR_OUT].type = IO_NONE;
        this->outputBridge->ops = new io_dev_ops();
        this->outputBridge->ops->write = &copyFromMFCBuffer;
        this->outputBridge->ops->req_bufs = func_req_bufs;
        this->outputBridge->ops->enq_buf = func_enq_buf;
        this->outputBridge->ops->deq_buf = func_deq_buf;

        if (dev_bufs_create(this->inputBridge, this->mfc, MFC_ENC_IN_NBUF)) {
            ROS_FATAL("Failed to input buffers on MFC");
        }

        if (dev_bufs_create(this->mfc, this->outputBridge, MFC_ENC_OUT_NBUF)) {
            ROS_FATAL("Failed to output buffers on MFC");
        }

        this->deviceChain[0] = this->inputBridge;
        this->deviceChain[1] = this->mfc;
        this->deviceChain[2] = this->outputBridge;
    }

    ~ExynosMultiFormatCodecH264Encoder() override
    {
        func_destroy(inputBridge);
        func_destroy(outputBridge);
        mfc->ops->destroy(mfc);
    }
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
        if (outputPixelFormat == PIX_FMT_YUV420P) {
            image->format = "yuv420p";
        }
        else if (outputPixelFormat == PIX_FMT_YUVJ422P) {
            image->format = "yuvj422p";
        }
        else {
            ROS_ERROR("Unknown pixel format");
        }
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

