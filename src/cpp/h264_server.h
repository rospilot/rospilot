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
#ifndef ROSPILOT_H264_SERVER_H
#define ROSPILOT_H264_SERVER_H

#include<mutex>
#include<condition_variable>
#include<vector>
#include<chrono>
#include<map>

#include<microhttpd.h>
#include<sensor_msgs/CompressedImage.h>

#include<image_sink.h>

namespace rospilot {

using namespace std::chrono;

struct ClientSession
{
    ClientSession()
    {
        keyFrame = false;
        lastAccessTime = high_resolution_clock::now();
    }
    std::vector<uint8_t> frameData;
    bool keyFrame;
    time_point<high_resolution_clock> lastAccessTime;
};

class H264Server : public ImageSink
{
private:
    std::condition_variable frameAvailable;
    std::mutex lock;
    std::map<std::string, ClientSession> clients;
    MHD_Daemon *daemon = nullptr;

public:
    // thread-safe
    void addFrame(sensor_msgs::CompressedImage *image, bool keyFrame) override;

    // thread-safe
    MHD_Response* readFrames(std::string client);

    // thread-safe
    void start();

    // thread-safe
    void stop();

    ~H264Server();
};

}

#endif
