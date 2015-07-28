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
#ifndef ROSPILOT_H264_SETTINGS_H
#define ROSPILOT_H264_SETTINGS_H

namespace rospilot {

enum H264Profile
{
    CONSTRAINED_BASELINE,
    HIGH
};

struct H264Settings
{
    int height;
    int width;
    int level;
    int gop_size;
    int bit_rate;
    bool zero_latency;
    H264Profile profile;
};

}

#endif
