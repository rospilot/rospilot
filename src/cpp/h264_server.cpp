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
#include "h264_server.h"

#include<ros/ros.h>
#include<microhttpd.h>
#include<sensor_msgs/CompressedImage.h>

namespace rospilot {

using std::vector;
using sensor_msgs::CompressedImage;

#define PORT 8666

int static handleRequest(void *custom,
        MHD_Connection *connection,
        const char *url,
        const char *method,
        const char *version,
        const char *uploadData,
        size_t *uploadDataSize,
        void **session)
{
    H264Server *server = (H264Server*) custom;
    MHD_Response *response;
    if (strncmp(url, "/h264/", 6) == 0) {
        response = server->readFrames(url);
    }
    else if (strncmp(url, "/h264_sps_pps", 13) == 0) {
        response = server->readSPSAndPPS();
    }
    else {
        ROS_ERROR("Bad URL requested: %s", url);
    }
    int ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
    MHD_destroy_response(response);
    return ret;
}

void H264Server::addFrame(sensor_msgs::CompressedImage *image, bool keyFrame)
{
    std::unique_lock<std::mutex> guard(lock);
    time_point<high_resolution_clock> currentTime = high_resolution_clock::now();
    // Purge clients that haven't accessed the stream in 10secs
    for (auto iter = clients.begin(); iter != clients.end(); ) {
        duration<double> duration = currentTime - iter->second.lastAccessTime;
        if (duration.count() > 10) {
            iter = clients.erase(iter);
        }
        else {
            iter++;
        }
    }

    if (keyFrame) {
        latestKeyFrame = image->data;
    }

    // Add data to all the clients, so they can fetch at their own pace
    for (auto &entry : clients) {
        if (keyFrame || !entry.second.keyFrame) {
            entry.second.frameData.clear();
        }
        if (entry.second.frameData.size() > 0) {
            continue;
        }
        entry.second.frameData = image->data;
        entry.second.keyFrame = keyFrame;
    }
    frameAvailable.notify_all();
}

MHD_Response* H264Server::readSPSAndPPS()
{
    std::unique_lock<std::mutex> guard(lock);
    std::vector<uint8_t> data;
    data.insert(data.end(), this->sps.begin(), this->sps.end());
    data.insert(data.end(), this->pps.begin(), this->pps.end());
    MHD_Response *response =
        MHD_create_response_from_buffer(data.size(),
                (void *) data.data(),
                MHD_RESPMEM_MUST_COPY);
    MHD_add_response_header(response, "Content-Type", "video/h264_sps_pps");
    // TODO: Change this to only be localhost and the local hostname
    MHD_add_response_header(response, "Access-Control-Allow-Origin", "*");
    return response;
}

MHD_Response* H264Server::readFrames(std::string client)
{
    std::unique_lock<std::mutex> guard(lock);
    if (clients.count(client) == 0) {
        // Send new clients the latest key frame
        clients[client].frameData = latestKeyFrame;
    }
    if (clients[client].frameData.size() == 0) {
        // Wait up to 100ms to see if a frame arrives
        frameAvailable.wait_for(guard, duration<double>(0.1));
    }
    ClientSession &session = clients[client];
    MHD_Response *response =
        MHD_create_response_from_buffer(session.frameData.size(),
                (void *) session.frameData.data(),
                MHD_RESPMEM_MUST_COPY);
    MHD_add_response_header(response, "Content-Type", "video/h264");
    // TODO: Change this to only be localhost and the local hostname
    MHD_add_response_header(response, "Access-Control-Allow-Origin", "*");
    session.frameData.clear();
    session.keyFrame = false;
    session.lastAccessTime = high_resolution_clock::now();
    return response;
}

void H264Server::setSPSAndPPS(std::vector<uint8_t> sps_data, std::vector<uint8_t> pps_data)
{
    std::lock_guard<std::mutex> guard(lock);
    this->sps.insert(sps.begin(), sps_data.begin(), sps_data.end());
    this->pps.insert(pps.begin(), pps_data.begin(), pps_data.end());
}

void H264Server::start()
{
    std::lock_guard<std::mutex> guard(lock);
    if (daemon != nullptr) {
        return;
    }
    daemon = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY,
            PORT,
            nullptr,
            nullptr,
            &handleRequest,
            this,
            // Support up to 8 clients
            MHD_OPTION_THREAD_POOL_SIZE, 8,
            // Since this is a video stream, the data is unlikely to be useful to the client after 1sec
            MHD_OPTION_CONNECTION_TIMEOUT, 1,
            MHD_OPTION_END);
}

void H264Server::stop()
{
    std::lock_guard<std::mutex> guard(lock);
    if (daemon == nullptr) {
        return;
    }
    MHD_stop_daemon(daemon);
    daemon = nullptr;
}

H264Server::~H264Server()
{
    stop();
}

}
