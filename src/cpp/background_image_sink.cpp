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

#include<sensor_msgs/CompressedImage.h>

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

void BackgroundImageSink::addFrame(sensor_msgs::CompressedImage const *input)
{
    // ignore call if the sink is still processing the last frame
    if (!sinkFuture.valid()) {
        return;
    }
    sensor_msgs::CompressedImage *imageCopy = new sensor_msgs::CompressedImage();
    *imageCopy = *input;
    sinkFuture = std::async(
            std::launch::async, 
            [this](sensor_msgs::CompressedImage *image){
                bool keyFrame = false;
                bool transcoded = true;
                if (resizer != nullptr) {
                    resizer->resizeInPlace(image);
                }
                if (encoder != nullptr) {
                    transcoded = encoder->encodeInPlace(image, &keyFrame);
                }
                if (transcoded) {
                    sink->addFrame(image, keyFrame);
                }
                delete image;
            },
            imageCopy);
}

BackgroundImageSink::~BackgroundImageSink()
{
    if (encoder != nullptr) {
        delete encoder;
    }
    if (resizer != nullptr) {
        delete resizer;
    }
}

