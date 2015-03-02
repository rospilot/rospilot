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

#include<chrono>

#include<sensor_msgs/CompressedImage.h>

extern "C" {
#include <libavcodec/avcodec.h>
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
    SoftwareVideoRecorder(int width, int height, PixelFormat pixelFormat, CodecID codecId);
    
    void writeFrame(sensor_msgs::CompressedImage *image, bool keyFrame);

    bool start(const char *name);
    
    bool stop();

    ~SoftwareVideoRecorder();

private:
    AVStream *createVideoStream(AVFormatContext *oc);
};

#endif
