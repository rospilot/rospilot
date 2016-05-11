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
#include "people_detector.h"

#include<ros/ros.h>
#include<sensor_msgs/CompressedImage.h>
#include<rospilot/VisionTargets.h>

namespace rospilot {

using std::vector;
using sensor_msgs::CompressedImage;

PeopleDetector::PeopleDetector(ros::Publisher *topic, int width, int height)
{
    this->detectedPeopleTopic = topic;
    this->width = width;
    this->height = height;
    hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
    handle = tjInitDecompress();
    context = sws_getContext(width, height, PIX_FMT_YUV420P,
                             width, height, PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
}

void PeopleDetector::addFrame(sensor_msgs::CompressedImage *image, bool keyFrame)
{
    std::vector<uint8_t> bgrData;
    bgrData.reserve(tjPixelSize[TJPF_BGR] * width * height);

    // TODO: use turbojpeg instead. Requires turbojpeg 1.4
//    int result = tjDecodeYUV(handle, image->data.data(),
//    0,
//    TJSAMP_420,
//    bgrData.data(),
//    width,
//    width * tjPixelSize[TJPF_BGR],
//    height,
//    TJPF_BGR,
//    0);
    auto base = image->data.data();
    uint8_t *inData[3] = { base, base + width * height, base + width * height * 5 / 4 }; // YUV420 has three plane
    uint8_t *outData[1] = { bgrData.data() }; // BGR has one plane
    int outLinesize[1] = { 3 * width }; // RGB stride
    int inLinesize[3] = { width, width / 2, width / 2 }; // YUV420 stride
    sws_scale(context, inData, inLinesize, 0, height, outData, outLinesize);

//    if (result == -1) {
//        ROS_ERROR("Error converting YUV to BGR: %s", tjGetErrorStr());
//        return;
//    }

    cv::Mat frame(height, width, CV_8UC3, bgrData.data());
    std::vector<cv::Rect> candidates;
    hog.detectMultiScale(frame, candidates, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 2);

    VisionTargets people;
    for (size_t i = 0; i < candidates.size(); i++)
    {
        cv::Rect candidate = candidates[i];

        bool contained = false;
        for (size_t j = 0; j < candidates.size(); j++) {
            if (j != i && (candidate & candidates[j]) == candidate) {
                contained = true;
                break;
            }
        }

        if (!contained) {
            VisionTarget person;
            person.id = i;
            int centerX = candidate.x + candidate.width / 2;
            int centerY = candidate.y + candidate.height / 2;
            person.x = 2.0 * centerX / (double) frame.cols - 1.0;
            person.y = 1.0 - 2.0 * centerY / (double) frame.rows;
            person.description = "person";
            people.targets.push_back(person);
        }
    }
    detectedPeopleTopic->publish(people);
}

PeopleDetector::~PeopleDetector()
{
    tjDestroy(handle);
    sws_freeContext(context);
}
}
