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

#include "background_image_sink.h"

#include<ros/ros.h>
#include<sensor_msgs/CompressedImage.h>

namespace rospilot {

using namespace std::chrono;

BackgroundImageSink::BackgroundImageSink(
        ImageSink *_sink,
        H264Encoder *_encoder,
        Resizer *_resizer) :
    sink(_sink), encoder(_encoder), resizer(_resizer)
{
    std::promise<void> promise;
    promise.set_value();
    sinkFuture = promise.get_future();
}

static void asyncWorker(Resizer *_resizer,
                H264Encoder *_encoder,
                ImageSink *_sink,
                sensor_msgs::CompressedImage image) {
    bool keyFrame = false;
    bool transcoded = true;
    if (_resizer != nullptr) {
        _resizer->resizeInPlace(&image);
    }
    if (_encoder != nullptr) {
        transcoded = _encoder->encodeInPlace(&image, &keyFrame);
    }
    if (transcoded) {
        _sink->addFrame(&image, keyFrame);
    }
}

void BackgroundImageSink::addFrame(sensor_msgs::CompressedImage const *input)
{
    // ignore call if the sink is still processing the last frame
    if (!sinkFuture.valid()) {
        return;
    }
    // Access the future to create a happens-before relation
    sinkFuture.get();
    sensor_msgs::CompressedImage copy;
    copy.header = input->header;
    copy.data = input->data;
    // XXX: Use c_str() to force a copy, as operator=(const string&) is COW and just
    // copies the internal buffer address, which is then not thread-safe
    copy.format = input->format.c_str();
    if (copy.data.data() == input->data.data() ||
            copy.format.c_str() == input->format.c_str()) {
        ROS_FATAL("copy operator created a non-thread-safe copy");
    }
    sinkFuture = std::async(
            std::launch::async, 
            &asyncWorker,
            resizer,
            encoder,
            sink,
            copy);
}

BackgroundImageSink::~BackgroundImageSink()
{
    sinkFuture.wait();
    if (encoder != nullptr) {
        delete encoder;
    }
    if (resizer != nullptr) {
        delete resizer;
    }
}

}
