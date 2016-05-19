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
class Copter
{
    static get parameters()
    {
        return [new ng.core.Inject('$rostopic'), new ng.core.Inject('$rosservice')];
    }

    constructor($rostopic, $rosservice)
    {
        this.rc_channels = $rostopic('/rospilot/rcstate', 'rospilot/RCState')
            .map(message => message.channel);

        this.global_position = $rostopic('/rospilot/gpsraw', 'rospilot/GPSRaw');

        this.waypoint = $rostopic('/rospilot/waypoints', 'rospilot/Waypoints')
            .filter(message => message.waypoints.length > 0)
            .map(message => message.waypoints[0]);
        this.waypoint_service = $rosservice('/rospilot/set_waypoints', 'rospilot/SetWaypoints');

        this.status = $rostopic('/rospilot/basic_status', 'rospilot/BasicStatus');
        this.status_service = $rosservice('/rospilot/set_mode', 'rospilot/SetBasicMode');

        var imu = $rostopic('/rospilot/imuraw', 'rospilot/IMURaw');
        this.accelerometer = imu.map(message => message.accel);
        this.gyroscope = imu.map(message => message.gyro);
        this.magnetometer = imu.map(message => message.mag);
    }

    getRCChannels()
    {
        return this.rc_channels;
    }

    getGlobalPosition()
    {
        return this.global_position;
    }

    getWaypoint()
    {
        return this.waypoint;
    }

    setWaypoint(latitude, longitude)
    {
        var waypoints = {
            waypoints: [{
                'latitude': latitude,
                'longitude': longitude,
                'altitude': 5.0
            }]};
        this.waypoint_service(waypoints);
    }

    getStatus()
    {
        return this.status;
    }

    setArmed(armedStatus)
    {
        this.status_service({armed: armedStatus});
    }

    getAccelerometer()
    {
        return this.accelerometer;
    }

    getGyroscope()
    {
        return this.gyroscope;
    }

    getMagnetometer()
    {
        return this.magnetometer;
    }
}

class RosParam
{
    static get parameters()
    {
        return [new ng.core.Inject('ROS')];
    }

    constructor(ROS)
    {
        this.ROS = ROS;
    }

    get(key, callback)
    {
        var param = new ROSLIB.Param({ros: this.ROS, name: key});
        param.get(callback);
    }

    set(key, value)
    {
        var param = new ROSLIB.Param({ros: this.ROS, name: key});
        param.set(value);
    }
}
