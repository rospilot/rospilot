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

    constructor(Copter)
    {
        this.x = Copter.getAccelerometer()
            .map(accel => accel.x);
        this.y = Copter.getAccelerometer()
            .map(accel => accel.y);
        this.z = Copter.getAccelerometer()
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

    constructor(Copter)
    {
        this.x = Copter.getGyroscope()
            .map(gyro => gyro.x);
        this.y = Copter.getGyroscope()
            .map(gyro => gyro.y);
        this.z = Copter.getGyroscope()
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

    constructor(Copter)
    {
        this.x = Copter.getMagnetometer()
            .map(mag => mag.x);
        this.y = Copter.getMagnetometer()
            .map(mag => mag.y);
        this.z = Copter.getMagnetometer()
            .map(mag => mag.z);
    }
}

class RollGuageComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotrollguage',
            template: ''
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(Copter)
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
        Copter.getAttitude()
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
}

class CompassComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotcompass',
            template: ''
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(Copter)
    {
        var compass = document.getElementById("compass");
        var needle = null;
        var needle_translate = null;
        compass.addEventListener('load', function() {
            needle = compass.getSVGDocument().getElementById("needle");
            needle_translate = needle.getAttribute("transform");
        });
        Copter.getAttitude()
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
}

class AttitudeComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotattitude',
            template: "<div>Roll: {{roll | async | number:'1.4-4'}}, Pitch: {{pitch | async | number:'1.4-4'}}, Yaw: {{yaw | async | number:'1.4-4'}}</div>"
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(Copter)
    {
        this.roll = Copter.getAttitude()
            .map(attitude => attitude.roll);
        this.pitch = Copter.getAttitude()
            .map(attitude => attitude.pitch);
        this.yaw = Copter.getAttitude()
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

    constructor(Copter)
    {
        this.channels = Copter.getRCChannels();
    }
}

class WaypointComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotwaypoint',
            template: "<div>Lat: {{latitude | async | number:'1.1-5'}}, Lng: {{longitude | async | number:'1.1-5'}}, Alt: {{altitude | async | number:'1.1-1'}}m</div>"
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(Copter)
    {
        this.latitude = Copter.getWaypoint()
            .map(waypoint => waypoint.latitude);
        this.longitude = Copter.getWaypoint()
            .map(waypoint => waypoint.longitude);
        this.altitude = Copter.getWaypoint()
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

    constructor(Copter)
    {
        this.latitude = Copter.getGlobalPosition()
            .map(position => position.latitude);
        this.longitude = Copter.getGlobalPosition()
            .map(position => position.longitude);
    }
}

class MapComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotmap',
            template: ''
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(Copter)
    {
        var map = L.map('map', {
            crs: L.CRS.EPSG3857,
            center: {lat: 37.77, lng: -122.49},
            zoom: 10,
            contextmenu: true,
            contextmenuWidth: 150,
            contextmenuItems: [
                {
                    text: 'Set Waypoint',
                    callback: function(e) {
                        Copter.setWaypoint(e.latlng.lat, e.latlng.lng);
                    },
                }
            ],
        });
        var server_name = window.location.hostname;
        var mapnik_url = 'http://' + server_name + ':8086/ex/{z}/{x}/{y}.png';

        L.tileLayer(mapnik_url, {
            maxZoom: 18,
        }).addTo(map);

        var copterIcon = L.AwesomeMarkers.icon({
            prefix: 'fa',
            icon: 'plane',
            markerColor: 'red'
        });

        this.marker = L.marker([37.77, -122.49], {
            title: 'Multicopter',
            icon: copterIcon
        }).addTo(map);

        var waypointIcon = L.AwesomeMarkers.icon({
            prefix: 'fa',
            icon: 'flag',
            markerColor: 'green'
        });

        this.waypoint_marker = L.marker([37.77, -122.49], {
            title: 'Waypoint',
            icon: waypointIcon
        }).addTo(map);

        Copter.getWaypoint().subscribe((waypoint) => {
            var lat = waypoint.latitude;
            var lng = waypoint.longitude;
            this.waypoint_marker.setLatLng([lat, lng]);
        });

        Copter.getGlobalPosition().subscribe((position) => {
            this.marker.setLatLng([position.latitude, position.longitude]);
        });
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

    constructor(Copter)
    {
        this.copter = Copter;
    }

    clicked()
    {
        navigator.geolocation.getCurrentPosition((loc) => {
            this.copter.setWaypoint(loc.coords.latitude, loc.coords.longitude);
        });
    }
}

class AccelerometerGraphComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'accelerometergraph',
            template: ''
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(Copter)
    {
        Highcharts.setOptions({
            global: {
                useUTC: false
            }
        });

        var chart;
        $('#accel_z_chart').highcharts({
            chart: {
                type: 'spline',
                animation: false,
                marginRight: 10,
            },
            title: {
                text: 'Accelerometer'
            },
            xAxis: {
                type: 'datetime',
                tickPixelInterval: 150,
                minRange: 15000
            },
            yAxis: {
                title: {
                    text: 'Value'
                },
                plotLines: [{
                    value: 0,
                    width: 1,
                    color: '#808080'
                }]
            },
            tooltip: {
                formatter: function() {
                        return '<b>'+ this.series.name +'</b><br/>'+
                        Highcharts.dateFormat('%Y-%m-%d %H:%M:%S', this.x) +'<br/>'+
                        Highcharts.numberFormat(this.y, 2);
                }
            },
            legend: {
                enabled: false
            },
            exporting: {
                enabled: false
            },
            series: [{
                name: 'data',
                data: [],
            }]
        });

        var last_redraw = new Date().getTime();
        Copter.getAccelerometer().subscribe(accel => {
            if ($('#accel_z_chart').length > 0) {
                var series = $('#accel_z_chart').highcharts().series[0];
                var x = (new Date()).getTime();
                var redraw = x - last_redraw > 500;
                var extremes = series.xAxis.getExtremes();
                // Shift out the data if there's more than 15secs on-screen
                var shift = extremes.dataMax - extremes.dataMin > 15000;
                if (redraw) {
                    last_redraw = x;
                }
                series.addPoint([x, accel.z], redraw, shift);
            }
        });
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
