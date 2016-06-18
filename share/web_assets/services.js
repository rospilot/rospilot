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
        return [RosTopic, RosService];
    }

    constructor($rostopic, $rosservice)
    {
        this.rc_channels = $rostopic.getTopic('/rospilot/rcstate', 'rospilot/RCState')
            .map(message => message.channel);

        this.global_position = $rostopic.getTopic('/rospilot/gpsraw', 'rospilot/GPSRaw');

        this.waypoint = $rostopic.getTopic('/rospilot/waypoints', 'rospilot/Waypoints')
            .filter(message => message.waypoints.length > 0)
            .map(message => message.waypoints[0]);
        this.waypoint_service = $rosservice.getService('/rospilot/set_waypoints', 'rospilot/SetWaypoints');

        this.status = $rostopic.getTopic('/rospilot/basic_status', 'rospilot/BasicStatus');
        this.status_service = $rosservice.getService('/rospilot/set_mode', 'rospilot/SetBasicMode');

        var imu = $rostopic.getTopic('/rospilot/imuraw', 'rospilot/IMURaw');
        this.accelerometer = imu.map(message => message.accel);
        this.gyroscope = imu.map(message => message.gyro);
        this.magnetometer = imu.map(message => message.mag);
        this.attitude = $rostopic.getTopic('/rospilot/attitude', 'rospilot/Attitude');
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

    getAttitude()
    {
        return this.attitude;
    }
}

class RosTopic
{
    static get parameters()
    {
        return [RosLib];
    }

    constructor(ros)
    {
        this.ros = ros.getRos();
    }

    getTopic(topic, type)
    {
        var topic = new ROSLIB.Topic({
            ros: this.ros,
            name: topic,
            messageType: type
        });

        return Rx.Observable.create((observer) => {
            topic.subscribe(message => {
                observer.next(message);
            });
            // TODO: should return a dispose method, but ROSLIB
            // can only remove all subscribers, not individual ones.
        });
    }
}

class RosParam
{
    static get parameters()
    {
        return [new ng.core.Inject(RosLib)];
    }

    constructor(ROS)
    {
        this.ROS = ROS.getRos();
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

class RosService
{
    static get parameters()
    {
        return [RosLib];
    }

    constructor(ROS)
    {
        this.ros = ROS.getRos();
    }

    getService(service_name, type)
    {
        var service = new ROSLIB.Service({
            ros: this.ros,
            name: service_name,
            messageType: type
        });

        return (args, callback) => {
            if (typeof args === 'undefined') {
                args = {};
            }
            if (typeof callback === 'undefined') {
                callback = function(result) {};
            }
            service.callService(new ROSLIB.ServiceRequest(args), callback);
        };
    }
}

class RosLib
{
    constructor()
    {
        var url = 'ws://' + window.location.hostname + ':8088';
        this.ros = new ROSLIB.Ros({url: url});
    }

    getRos()
    {
        return this.ros;
    }
}

class OnboardComputer
{
    static get parameters()
    {
        return [Camera, ng.http.Http, RosService, RosParam, RosTopic];
    }

    constructor(camera, http, $rosservice, $rosparam, $rostopic)
    {
        this.shutdownService = $rosservice.getService('/rospilot/shutdown', 'std_srvs/Empty');
        this.camera = camera;
        this.http = http;
        this.rosparam = $rosparam;
        this.vision_targets = $rostopic.getTopic('/rospilot/camera/vision_targets', 'rospilot/VisionTargets');
        this.active_video_device = Rx.Observable.interval(1000)
            .flatMap(() => Rx.Observable.create((observer) => {
                $rosparam.get('/rospilot/camera/video_device', device => observer.next(device));
            }));
        this.media = Rx.Observable.interval(1000)
            .flatMap(() => {
                return http.get('/api/media/')
                .map(response => {
                    var objs = response.json();
                    for (let obj of objs) {
                        obj.delete = function() {
                            if (confirm("Are you sure?")) {
                                http.delete('/api/media?id=' + obj.id)
                                    .subscribe();
                            }
                        };
                    }
                    return objs;
                });
            });
    }

    shutdown()
    {
        this.shutdownService();
    }

    getVisionTargets()
    {
        return this.vision_targets;
    }

    getCamera()
    {
        return this.camera;
    }

    getMedia()
    {
        return this.media;
    }

    getVideoDevices()
    {
        return this.http.get('/api/video_devices/')
            .map(response => response.json());
    }

    getActiveVideoDevice()
    {
        return this.active_video_device;
    }

    setActiveVideoDevice(device)
    {
        this.rosparam.set('/rospilot/camera/video_device', device);
    }
}

class Camera
{
    static get parameters()
    {
        return [RosTopic, RosService, RosParam];
    }

    constructor($rostopic, $rosservice, $rosparam)
    {
        this.start_recording = $rosservice.getService('/rospilot/camera/start_record', 'std_srvs/Empty');
        this.stop_recording = $rosservice.getService('/rospilot/camera/stop_record', 'std_srvs/Empty');
        this.take_picture = $rosservice.getService('/rospilot/camera/capture_image', 'std_srvs/Empty');
        this.resolutions = $rostopic.getTopic('/rospilot/camera/resolutions', 'rospilot/Resolutions')
            .map(message => {
                var strings = [];
                for (let resolution of message.resolutions) {
                    strings.push(resolution.width + 'x' + resolution.height);
                }
                return strings;
            });
        this.rosparam = $rosparam;
        this.recording = false;
    }

    getRecording()
    {
        return this.recording;
    }

    startRecording()
    {
        this.recording = true;
        this.start_recording();
    }

    stopRecording()
    {
        this.recording = false;
        this.stop_recording();
    }

    takePicture()
    {
        this.take_picture();
    }

    getAllResolutions()
    {
        return this.resolutions;
    }

    getResolution()
    {
        return Rx.Observable.create((observer) => {
                this.rosparam.get('/rospilot/camera/resolution', resolution => observer.next(resolution));
            });
    }

    setResolution(resolution)
    {
        this.rosparam.set('/rospilot/camera/resolution', resolution);
    }
}
