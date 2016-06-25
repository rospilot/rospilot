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

    constructor(rostopic, rosservice)
    {
        this.rc_channels = rostopic.getTopic('/rospilot/rcstate', 'rospilot/RCState')
            .map(message => message.channel);

        this.global_position = rostopic.getTopic('/rospilot/gpsraw', 'rospilot/GPSRaw');

        this.waypoint = rostopic.getTopic('/rospilot/waypoints', 'rospilot/Waypoints')
            .filter(message => message.waypoints.length > 0)
            .map(message => message.waypoints[0]);
        this.waypoint_service = rosservice.getService('/rospilot/set_waypoints', 'rospilot/SetWaypoints');

        this.status = rostopic.getTopic('/rospilot/basic_status', 'rospilot/BasicStatus');
        this.status_service = rosservice.getService('/rospilot/set_mode', 'rospilot/SetBasicMode');

        var imu = rostopic.getTopic('/rospilot/imuraw', 'rospilot/IMURaw');
        this.accelerometer = imu.map(message => message.accel);
        this.gyroscope = imu.map(message => message.gyro);
        this.magnetometer = imu.map(message => message.mag);
        this.attitude = rostopic.getTopic('/rospilot/attitude', 'rospilot/Attitude');
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

class OnboardComputer
{
    static get parameters()
    {
        return [ng.http.Http, RosService, RosParam, RosTopic];
    }

    constructor(http, rosservice, rosparam, rostopic)
    {
        this.shutdownService = rosservice.getService('/rospilot/shutdown', 'std_srvs/Empty');
        this.http = http;
        this.rosparam = rosparam;
        this.vision_targets = rostopic.getTopic('/rospilot/camera/vision_targets', 'rospilot/VisionTargets');
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

    isComputerVisionEnabled()
    {
        return this.rosparam.get('/rospilot/camera/detector_enabled');
    }

    setComputerVision(enabled)
    {
        this.rosparam.set('/rospilot/camera/detector_enabled', enabled);
    }

    getVisionTargets()
    {
        return this.vision_targets;
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
        return this.rosparam.get('/rospilot/camera/video_device');
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

    constructor(rostopic, rosservice, rosparam)
    {
        this.start_recording = rosservice.getService('/rospilot/camera/start_record', 'std_srvs/Empty');
        this.stop_recording = rosservice.getService('/rospilot/camera/stop_record', 'std_srvs/Empty');
        this.take_picture = rosservice.getService('/rospilot/camera/capture_image', 'std_srvs/Empty');
        this.resolutions = rostopic.getTopic('/rospilot/camera/resolutions', 'rospilot/Resolutions')
            .map(message => {
                var strings = [];
                for (let resolution of message.resolutions) {
                    strings.push(resolution.width + 'x' + resolution.height);
                }
                return strings;
            });
        this.rosparam = rosparam;
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
        return this.rosparam.get('/rospilot/camera/resolution');
    }

    setResolution(resolution)
    {
        this.rosparam.set('/rospilot/camera/resolution', resolution);
    }
}

class VideoStream
{
    static get parameters()
    {
        return [ng.http.Http];
    }

    constructor(http)
    {
        this.resolution = new Rx.Subject();
        var videoWidth = 640;
        var videoHeight = 480;
        this.fps = new Rx.Subject();
        var fpsStartTime = new Date().getTime();
        var frameCount = 0;
        this.player = new Player({size: {width: videoWidth, height: videoHeight}});
        this.canvas_subscribers = 0;

        this.player.onPictureDecoded = (data, width, height) => {
            if (width != videoWidth || height != videoHeight) {
                this.resolution.next({width: width, height: height});
                videoWidth = width;
                videoHeight = height;
            }
            frameCount++;
            var currentTime = new Date().getTime();
            var deltaSeconds = (currentTime - fpsStartTime) / 1000.0
            if (deltaSeconds > 1) {
                this.fps.next(Math.floor(frameCount / deltaSeconds));
                frameCount = 0;
                fpsStartTime = currentTime;
            }
        };

        var server_name = window.location.hostname;
        // Generate a random client id for fetching the stream
        var clientId = Math.floor(Math.random() * 1000 * 1000 * 1000);
        this.url = 'http://' + server_name + ':8666/h264/' + clientId;
    }

    nextFrame()
    {
        let req = new XMLHttpRequest();
        req.open('get', this.url);
        req.responseType = "arraybuffer";
        req.onreadystatechange = () => {
            if (req.readyState == 4) {
                if (req.status == 200) {
                    this.player.decode(new Uint8Array(req.response));
                    if (this.canvas_subscribers > 0) {
                        this.timeout_id = setTimeout(() => this.nextFrame(), 1);
                    }
                }
                else {
                    if (this.canvas_subscribers > 0) {
                        this.timeout_id = setTimeout(() => this.nextFrame(), 1000);
                    }
                }
            }
        };
        req.send();
        // TODO: the below code should work, but data.arrayBuffer() isn't implemented in Angular 2 yet
        //http.get(h264Url)
        //    .subscribe(data => {
        //        player.decode(new Uint8Array(data.arrayBuffer()));
        //        setTimeout(nextFrame, 1);
        //    },
        //    () => {
        //        setTimeout(nextFrame, 1000);
        //    });
    }

    getFPS()
    {
        return this.fps;
    }

    getResolution()
    {
        return this.resolution;
    }

    getCanvas()
    {
        return Rx.Observable.create(observer => {
            this.canvas_subscribers++;
            if (this.canvas_subscribers == 1) {
                // Start fetching frames for the first subscriber
                this.nextFrame();
            }
            observer.next(this.player.canvas);
            return () => {
                this.canvas_subscribers--;
                if (this.canvas_subscribers == 0) {
                    clearTimeout(this.timeout_id);
                }
            };
        });
    }
}
