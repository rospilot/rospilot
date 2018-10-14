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
#include<vector>

#include<ros/ros.h>

namespace rospilot {

int nextNALStart(std::vector<uint8_t> &data, size_t start, uint8_t *nalType)
{
    for (size_t i = start; i + 2 < data.size(); i++) {
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

void tryExtractSPSandPPS(std::vector<uint8_t> &data, 
        std::vector<uint8_t> &sps_out,
        std::vector<uint8_t> &pps_out)
{
    uint8_t nalType;
    for (int i = nextNALStart(data, 0, &nalType); i != -1 && i + 3 < (int) data.size();) {
        uint8_t nextNalType;
        int j = nextNALStart(data, i + 4, &nextNalType);
        if (j == -1) {
            j = data.size();
        }
        if ((nalType & 0x1f) == 7) {
            sps_out.insert(sps_out.begin(), data.begin() + i, data.begin() + j);
            ROS_INFO("Found SPS");
        }
        if ((nalType & 0x1f) == 8) {
            pps_out.insert(pps_out.begin(), data.begin() + i, data.begin() + j);
            ROS_INFO("Found PPS");
        }
        i = j;
        nalType = nextNalType;
    }
}

}
