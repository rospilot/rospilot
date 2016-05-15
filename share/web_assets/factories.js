/*
 * Copyright 2012 the original author or authors.
 * See the NOTICE file distributed with this work for additional
 * information regarding copyright ownership.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
angular.module('rospilot')
.factory('ROS', function() {
    var url = 'ws://' + window.location.hostname + ':8088';
    var ros = new ROSLIB.Ros({url: url});
    return ros;
})
.factory('$rosservice', function(ROS) {
    return function(service_name, type) {
        var service = new ROSLIB.Service({
            ros : ROS,
            name : service_name,
            messageType : type
        });
        return function(args, callback) {
            if (typeof args === 'undefined') {
                args = {};
            }
            if (typeof callback === 'undefined') {
                callback = function(result) {};
            }
            service.callService(new ROSLIB.ServiceRequest(args), callback);
        };
    };
})
.factory('$rosparam', function(ROS) {
    return {
        get: function(key, callback) {
            var param = new ROSLIB.Param({ros: ROS, name: key});
            param.get(callback);
        },
        set: function(key, value) {
            var param = new ROSLIB.Param({ros: ROS, name: key});
            param.set(value);
        }
    };
})
.factory('$rostopic', function(ROS) {
    return function(topic, type) {
        return new ROSLIB.Topic({
            ros : ROS,
            name : topic,
            messageType : type
        });
    };
})
.factory('Camera', function ($rosservice, $rostopic) {
    return {
        resolutions: $rostopic('/rospilot/camera/resolutions', 'rospilot/Resolutions'),
        take_picture: $rosservice('/rospilot/camera/capture_image', 'std_srvs/Empty'),
        start_recording: $rosservice('/rospilot/camera/start_record', 'std_srvs/Empty'),
        stop_recording: $rosservice('/rospilot/camera/stop_record', 'std_srvs/Empty')
    };
})
.factory('Status', function ($rostopic, $rosservice) {
      return {
          subscribe: function(callback) {
              $rostopic('/rospilot/basic_status', 'rospilot/BasicStatus').subscribe(callback);
          },
          set: $rosservice('/rospilot/set_mode', 'rospilot/SetBasicMode')
      };
})
.factory('VisionTargets', function ($rostopic) {
      return $rostopic('/rospilot/camera/vision_targets', 'rospilot/VisionTargets');
})
.factory('Position', function ($rostopic) {
      return $rostopic('/rospilot/gpsraw', 'rospilot/GPSRaw');
})
.factory('Attitude', function ($rostopic) {
      return $rostopic('/rospilot/attitude', 'rospilot/Attitude');
})
.factory('RCState', function ($rostopic) {
      return $rostopic('/rospilot/rcstate', 'rospilot/RCState');
})
.factory('IMU', function ($rostopic) {
      return $rostopic('/rospilot/imuraw', 'rospilot/IMURaw');
})
.factory('Waypoints', function ($rostopic, $rosservice) {
      return {
          subscribe: function(callback) {
              $rostopic('/rospilot/waypoints', 'rospilot/Waypoints').subscribe(callback);
          },
          set: $rosservice('/rospilot/set_waypoints', 'rospilot/SetWaypoints')
      };
});
