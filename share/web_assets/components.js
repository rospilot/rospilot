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
class StatusComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotstatus',
            templateUrl: '/static/status_component.html'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.copter = copter;
        this.flight_mode = copter.getStatus()
            .map(status => status.flight_mode);
        this.armed = copter.getStatus()
            .map(status => status.armed);
    }

    disarm()
    {
        this.copter.setArmed(false);
    }

    arm()
    {
        this.copter.setArmed(true);
    }
}

class AccelerometerClippingComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotaccelerometerclipping',
            template: '<div>Accel clipping: IMU0: {{accel_0 | async}}, IMU1: {{accel_1 | async}}, IMU2: {{accel_2 | async}}</div>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.accel_0 = copter.getAccelerometerClippingCounts()
            .map(clipping => clipping[0]);
        this.accel_1 = copter.getAccelerometerClippingCounts()
            .map(clipping => clipping[1]);
        this.accel_2 = copter.getAccelerometerClippingCounts()
            .map(clipping => clipping[2]);
    }
}

class AccelerometerComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotaccelerometer',
            template: '<div>Accel: x: {{x | async}}, y: {{y | async}}, z: {{z | async}}</div>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.x = copter.getAccelerometer()
            .map(accel => accel.x);
        this.y = copter.getAccelerometer()
            .map(accel => accel.y);
        this.z = copter.getAccelerometer()
            .map(accel => accel.z);
    }
}

class GyroscopeComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotgyroscope',
            template: '<div>Gyro: x: {{x | async}}, y: {{y | async}}, z: {{z | async}}</div>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.x = copter.getGyroscope()
            .map(gyro => gyro.x);
        this.y = copter.getGyroscope()
            .map(gyro => gyro.y);
        this.z = copter.getGyroscope()
            .map(gyro => gyro.z);
    }
}

class MagnetometerComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotmagnetometer',
            template: '<div>Mag: x: {{x | async}}, y: {{y | async}}, z: {{z | async}}</div>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.x = copter.getMagnetometer()
            .map(mag => mag.x);
        this.y = copter.getMagnetometer()
            .map(mag => mag.y);
        this.z = copter.getMagnetometer()
            .map(mag => mag.z);
    }
}

class BatteryComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotbattery',
            template: "<div>Battery: {{voltage | async | number:'1.2-2'}}V</div>"
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.voltage = copter.getBattery()
            .map(battery => battery.voltage);
    }
}

class VibrationComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotvibration',
            template: `<div>Vibration:
              x: <span [ngStyle]="x_style | async">{{x | async | number:'1.2-2'}}m/s<sup>2</sup></span>
              y: <span [ngStyle]="y_style | async">{{y | async | number:'1.2-2'}}m/s<sup>2</sup></span>
              z: <span [ngStyle]="z_style | async">{{z | async | number:'1.2-2'}}m/s<sup>2</sup></span>
            </div>`
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    static qualityThreshold(value)
    {
        if (value < 15) {
            return {"color": "green"};
        }
        else if (value < 30) {
            return {"color": "yellow"};
        }
        else {
            return {"color": "red"};
        }
    }

    constructor(copter)
    {
        this.x = copter.getVibration()
            .map(vibration => vibration.x);
        this.x_style = this.x
            .map(value => VibrationComponent.qualityThreshold(value));
        this.y = copter.getVibration()
            .map(vibration => vibration.y);
        this.y_style = this.y
            .map(value => VibrationComponent.qualityThreshold(value));
        this.z = copter.getVibration()
            .map(vibration => vibration.z);
        this.z_style = this.z
            .map(value => VibrationComponent.qualityThreshold(value));
    }
}

class RollGuageComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotrollguage',
            template: '<object id="attitude_svg" data="/static/attitude.svg" type="image/svg+xml"></object>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.copter = copter;
    }

    ngOnInit()
    {
        var attitude_svg = document.getElementById("attitude_svg");
        var roll_gauge = null;
        var roll_gauge_translate = null;
        var roll_needle = null;
        attitude_svg.addEventListener('load', function() {
            roll_needle = attitude_svg.getSVGDocument().getElementById("layer2");
            roll_gauge = attitude_svg.getSVGDocument().getElementById("layer5");
            roll_gauge_translate = roll_gauge.getAttribute("transform");
        });
        this.subscription = this.copter.getAttitude()
            .subscribe(attitude => {
                if (roll_gauge != null) {
                    var x = roll_needle.getBBox().width / 2.0;
                    var y = roll_needle.getBBox().height / 2.0;
                    var roll = -attitude.roll * 180 / Math.PI;
                    var pitch = attitude.pitch * roll_gauge.getBBox().height / Math.PI;
                    roll_needle.setAttribute("transform",
                        "rotate(" + roll + " " + x + " " + y + ")");
                    roll_gauge.setAttribute("transform",
                        "rotate(" + roll + " " + x + " " + y + ") " +
                        roll_gauge_translate + " translate(0 " + pitch + ")");
                }
            });
    }

    ngOnDestroy()
    {
        this.subscription.unsubscribe();
    }
}

class CompassComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotcompass',
            template: '<object id="compass" data="/static/compass.svg" type="image/svg+xml"></object>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.copter = copter;
    }

    ngOnInit()
    {
        var compass = document.getElementById("compass");
        var needle = null;
        var needle_translate = null;
        compass.addEventListener('load', function() {
            needle = compass.getSVGDocument().getElementById("needle");
            needle_translate = needle.getAttribute("transform");
        });
        this.subscription = this.copter.getAttitude()
            .subscribe(attitude => {
                if (needle != null) {
                    var x = needle.getBBox().width / 2.0;
                    var y = needle.getBBox().height / 2.0;
                    var yaw = attitude.yaw * 180 / Math.PI;
                    needle.setAttribute("transform",
                        "rotate(" + -yaw + " " + x + " " + y + ") "
                        + needle_translate);
                }
            });
    }

    ngOnDestroy()
    {
        this.subscription.unsubscribe();
    }
}

class AttitudeComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotattitude',
            template: `<div>
                Roll: {{roll | async | number:'1.4-4'}}, 
                Pitch: {{pitch | async | number:'1.4-4'}}, 
                Yaw: {{yaw | async | number:'1.4-4'}}
            </div>`
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.roll = copter.getAttitude()
            .map(attitude => attitude.roll);
        this.pitch = copter.getAttitude()
            .map(attitude => attitude.pitch);
        this.yaw = copter.getAttitude()
            .map(attitude => attitude.yaw);
    }
}

class RCStateComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rcstate',
            template: '<div>RC Channels: {{channels | async}}</div>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.channels = copter.getRCChannels();
    }
}

class WaypointComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotwaypoint',
            template: `<div>
                Lat: {{latitude | async | number:'1.1-5'}}, 
                Lng: {{longitude | async | number:'1.1-5'}}, 
                Alt: {{altitude | async | number:'1.1-1'}}m
                </div>`
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.latitude = copter.getWaypoint()
            .map(waypoint => waypoint.latitude);
        this.longitude = copter.getWaypoint()
            .map(waypoint => waypoint.longitude);
        this.altitude = copter.getWaypoint()
            .map(waypoint => waypoint.altitude);
    }
}

class GlobalPositionComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'globalposition',
            template: '<div>Lat: {{latitude | async}}, Lng: {{longitude | async}}</div>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.latitude = copter.getGlobalPosition()
            .map(position => position.latitude);
        this.longitude = copter.getGlobalPosition()
            .map(position => position.longitude);
    }
}

class ComeHereComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotcomehere',
            template: '<button type="button" class="btn" (click)="clicked()">Come To Me</button>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.copter = copter;
    }

    clicked()
    {
        navigator.geolocation.getCurrentPosition((loc) => {
            this.copter.setWaypoint(loc.coords.latitude, loc.coords.longitude);
        });
    }
}

class FollowMeComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotfollowme',
            template: '<button [ngClass]="style" type="button" class="btn" (click)="clicked()">Follow Me</button>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(copter)
    {
        this.copter = copter;
        this.following = false;
        this.timeout_id = null;
        this.style = "";
    }

    clicked()
    {
        this.following = !this.following;
        if (this.following) {
            this.updateWaypoint();
            this.style = "active";
        }
        else {
            clearTimeout(this.timeout_id);
            this.style = "";
        }
    }

    updateWaypoint()
    {
        navigator.geolocation.getCurrentPosition((loc) => {
            this.copter.setWaypoint(loc.coords.latitude, loc.coords.longitude);
        });
        this.timeout_id = setTimeout(() => this.updateWaypoint(), 1000);
    }
}

class RecordingButton
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotrecordingbutton',
            templateUrl: '/static/recording_button.html'
        })];
    }

    static get parameters()
    {
        return [Camera];
    }

    constructor(camera)
    {
        this.camera = camera;
    }

    clicked()
    {
        if (this.camera.getRecording()) {
            this.camera.stopRecording();
        }
        else {
            this.camera.startRecording();
        }
    }
}

class TakePictureButton
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'takepicturebutton',
            templateUrl: '/static/take_picture_button.html'
        })];
    }

    static get parameters()
    {
        return [Camera];
    }

    constructor(camera)
    {
        this.camera = camera;
    }
}

class MediaListComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotmedialist',
            templateUrl: '/static/media_list.html'
        })];
    }

    static get parameters()
    {
        return [OnboardComputer];
    }

    constructor(computer)
    {
        this.media = computer.getMedia();
    }
}

class ShutdownComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'shutdownbutton',
            template: `<button type="button" class="btn btn-danger" 
                (click)="clicked()">Shutdown On-board Computer</button>`
        })];
    }

    static get parameters()
    {
        return [OnboardComputer];
    }

    constructor(computer)
    {
        this.computer = computer;
    }

    clicked()
    {
        this.computer.shutdown();
    }
}

class VideoDevicesComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'videodevices',
            templateUrl: '/static/video_devices_component.html'
        })];
    }

    static get parameters()
    {
        return [OnboardComputer];
    }

    constructor(computer)
    {
        this.computer = computer;
        this.devices = computer.getVideoDevices();
        this.selected_device = computer.getActiveVideoDevice();
    }

    onSelected(device)
    {
        this.computer.setActiveVideoDevice(device);
    }
}

class CameraResolutionsComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'cameraresolutions',
            templateUrl: '/static/camera_resolutions_component.html'
        })];
    }

    static get parameters()
    {
        return [Camera];
    }

    constructor(camera)
    {
        this.camera = camera;
        this.resolutions = camera.getAllResolutions();
        this.selected_resolution = camera.getResolution();
    }

    onSelected(resolution)
    {
        this.camera.setResolution(resolution);
    }
}

class ComputerVisionToggle
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'computervisiontoggle',
            template: `Computer vision: <input type="checkbox" [ngModel]="enabled | async" 
                (ngModelChange)="setEnabled($event)">`
        })];
    }

    static get parameters()
    {
        return [OnboardComputer];
    }

    constructor(computer)
    {
        this.computer = computer;
        this.enabled = computer.isComputerVisionEnabled();
    }

    setEnabled(enable)
    {
        this.computer.setComputerVision(enable);
    }
}

class FPSDisplay
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'fpsdisplay',
            template: 'FPS: {{fps | async}}'
        })];
    }

    static get parameters()
    {
        return [VideoStream];
    }

    constructor(stream)
    {
        this.fps = stream.getFPS();
    }
}

class VideoDisplay
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'videodisplay',
            template: '<div id="video" style="cursor: pointer;" (click)="camera.takePicture()"></div>'
        })];
    }

    static get parameters()
    {
        return [VideoStream, Camera, OnboardComputer];
    }

    constructor(stream, camera, computer)
    {
        this.camera = camera;
        this.stream = stream;
        this.computer = computer;
    }

    ngOnInit()
    {
        var renderer = PIXI.autoDetectRenderer(640, 480, {transparent: true});
        this.resolution_subscription = this.stream.getResolution().subscribe(resolution => {
            renderer.resize(resolution.width, resolution.height);
            document.getElementById('video').style.width = resolution.width + "px";
            document.getElementById('video').style.height = resolution.height + "px";
        });

        document.getElementById('video').appendChild(renderer.view);
        renderer.view.style.zIndex = "2";

        // create the root of the scene graph
        var stage = new PIXI.Container();
        var textObjs = new Map();
        this.vision_subscription = this.computer.getVisionTargets().subscribe(function(message) {
            var targetIds = new Set();
            for (let target of message.targets) {
                targetIds.add(target.id);
                if (!textObjs.has(target.id)) {
                    textObjs.set(target.id, new PIXI.Text(target.description, {fill: 'red'}));
                    stage.addChild(textObjs.get(target.id));
                }
                var textObj = textObjs.get(target.id);
                textObj.visible = true;
                textObj.x = renderer.width * (target.x + 1) / 2.0;
                textObj.y = renderer.height * (1 - target.y) / 2.0;
            }
            for (let [key, value] of textObjs) {
                if (!targetIds.has(key)) {
                    // Hide element rather than removing it. Because adding/removing is more expensive
                    value.visible = false;
                }
            }
            renderer.render(stage);
        });

        this.canvas_subscription = this.stream.getCanvas().subscribe(canvas =>{
            document.getElementById('video').appendChild(canvas);
            canvas.style.zIndex = "1";
        });
    }

    ngOnDestroy()
    {
        this.resolution_subscription.unsubscribe();
        this.vision_subscription.unsubscribe();
        this.canvas_subscription.unsubscribe();
    }
}

class RospilotApp
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotapp',
            templateUrl: '/static/app.html',
        })];
    }

    static get parameters()
    {
        return [];
    }
}

class FlightControlPage
{
    static get annotations()
    {
        return [new ng.core.Component({
            templateUrl: '/static/flight_control.html',
        })];
    }

    static get parameters()
    {
        return [];
    }
}

class CameraPage
{
    static get annotations()
    {
        return [new ng.core.Component({
            templateUrl: '/static/camera.html',
        })];
    }

    static get parameters()
    {
        return [];
    }
}

class SettingsPage
{
    static get annotations()
    {
        return [new ng.core.Component({
            templateUrl: '/static/settings.html',
        })];
    }

    static get parameters()
    {
        return [];
    }
}
