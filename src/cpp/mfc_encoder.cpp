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

#include<vector>

#include<ros/ros.h>
#include<sensor_msgs/CompressedImage.h>

extern "C" {
#include <linux/videodev2.h>
#include <rospilot_deps/mfc/mfc.h>
#include <rospilot_deps/mfc/io_dev.h>
#include <rospilot_deps/mfc/func_dev.h>
#include <rospilot_deps/mfc/v4l_dev.h>
}

using std::vector;
using sensor_msgs::CompressedImage;

struct OutputBridgePriv
{
    vector<unsigned char> *image;
    bool readSuccessful;
};

bool ExynosMultiFormatCodecH264Encoder::encodeInPlace(sensor_msgs::CompressedImage *image,
        bool *keyFrame)
{
    if (image->format != "yuv420p") {
        ROS_ERROR("Image is not in yuv420p format");
        return false;
    }

    inputBridge->priv = &image->data;
    OutputBridgePriv *outPriv = (OutputBridgePriv *)outputBridge->priv;
    outPriv->image = &image->data;
    outPriv->readSuccessful = false;

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

    // XXX: This relies on the flags being in the mfc's private section still.
    // This really needs to be refactored at some point.
    int flags = ((mfc_priv *) this->mfc->priv)->last_frame_flags;
    *keyFrame = (flags & V4L2_BUF_FLAG_KEYFRAME);

    if (outPriv->readSuccessful) {
        image->format = "h264";
        if (sps.size() == 0 || pps.size() == 0) {
            tryExtractSPSandPPS(image->data);
        }
        // Insert the PPS and SPS if this is a keyframe.
        // This ensures that those frames are completely independent for streaming
        // and in case the receiver (recorder) picks up in the middle of a stream
        if (*keyFrame) {
            image->data.insert(image->data.begin(), sps.begin(), sps.end());
            image->data.insert(image->data.begin(), pps.begin(), pps.end());
        }
    }

    return outPriv->readSuccessful;
}

int nextNALStart(std::vector<uint8_t> &data, int start, uint8_t *nalType)
{
    for (int i = start; i + 2 < data.size(); i++) {
        if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
            if (i + 3 < data.size()) {
                *nalType = data[i + 3];
            }
            else {
                ROS_WARN("Expected NAL type");
            }
            if (i - 1 >= 0 && data[i - 1] == 0) {
                return i - 1;
            }
            return i;
        }
    }
    return -1;
}

void ExynosMultiFormatCodecH264Encoder::tryExtractSPSandPPS(std::vector<uint8_t> &data)
{
    uint8_t nalType;
    for (int i = nextNALStart(data, 0, &nalType); i != -1 && i + 3 < data.size();) {
        uint8_t nextNalType;
        int j = nextNALStart(data, i + 4, &nextNalType);
        if (j == -1) {
            j = data.size();
        }
        if ((nalType & 0x1f) == 7) {
            sps.insert(sps.begin(), data.begin() + i, data.begin() + j);
            ROS_INFO("Found SPS");
        }
        if ((nalType & 0x1f) == 8) {
            pps.insert(pps.begin(), data.begin() + i, data.begin() + j);
            ROS_INFO("Found PPS");
        }
        i = j;
        nalType = nextNalType;
    }
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
    memcpy(bufs[0], image->data(), lens[0]);
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
    image->reserve(lens[0]);
    image->insert(image->begin(), bufs[0], bufs[0] + lens[0]);
    
    return 0;
}

ExynosMultiFormatCodecH264Encoder::ExynosMultiFormatCodecH264Encoder(std::string path, int width, int height)
{
    this->mfc = mfc_create(path.c_str());
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

ExynosMultiFormatCodecH264Encoder::~ExynosMultiFormatCodecH264Encoder()
{
    func_destroy(inputBridge);
    func_destroy(outputBridge);
    mfc->ops->destroy(mfc);
}
