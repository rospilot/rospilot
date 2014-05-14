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
angular.module('rospilot', ['ngRoute', 'ngResource'])
.config(function($routeProvider) {
    $routeProvider
    .when("/graphs", {templateUrl:'static/graphs.html', controller:'graphs'})
    .when("/camera", {templateUrl:'static/camera.html', controller:'camera'})
    .when("/settings", {templateUrl:'static/settings.html', controller:'settings'})
    .when("/flight_control", {templateUrl: 'static/flight_control.html', controller: 'position'})
    .otherwise({redirectTo:"/flight_control"});
})
.factory('ROS', function() {
    var url = 'ws://' + window.location.hostname + ':8088';
    var ros = new ROSLIB.Ros({url: url});
    return ros;
})
.factory('$rosservice', function(ROS) {
    return function(service, type) {
        var service = new ROSLIB.Service({
            ros : ROS,
            name : service,
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
.factory('Camera', function ($rosservice) {
    return {
        take_picture: $rosservice('/take_picture', 'std_srvs/Empty'),
        start_recording: $rosservice('/start_record', 'std_srvs/Empty'),
        stop_recording: $rosservice('/stop_record', 'std_srvs/Empty')
    };
})
.factory('Media', function ($resource) {
      return $resource('api/media');
})
.factory('Status', function ($rostopic, $rosservice) {
      return {
          subscribe: function(callback) {
              $rostopic('/basic_status', 'rospilot/BasicStatus').subscribe(callback);
          },
          set: $rosservice('/set_mode', 'rospilot/SetBasicMode')
      };
})
.factory('Position', function ($rostopic) {
      return $rostopic('/gpsraw', 'rospilot/GPSRaw');
})
.factory('Attitude', function ($rostopic) {
      return $rostopic('/attitude', 'rospilot/Attitude');
})
.factory('RCState', function ($rostopic) {
      return $rostopic('/rcstate', 'rospilot/RCState');
})
.factory('IMU', function ($rostopic) {
      return $rostopic('/imuraw', 'rospilot/IMURaw');
})
.factory('Waypoints', function ($rostopic, $rosservice) {
      return {
          subscribe: function(callback) {
              $rostopic('/waypoints', 'rospilot/Waypoints').subscribe(callback);
          },
          set: $rosservice('/set_waypoints', 'rospilot/SetWaypoints')
      };
})
.controller('rospilot', function ($scope, $route) {
    $scope.$route = $route;
})
.controller('settings', function ($scope, $rosparam, $rosservice) {
    var globService = $rosservice('/glob', 'rospilot/Glob');
    $scope.selected_video_device = '';
    $scope.video_devices = [];
    $rosparam.get('/camera/video_device',
        function(value) {
            $scope.selected_video_device = value;
            // XXX: This shouldn't be necessary
            $scope.$apply();
        }
    );
    globService({pattern: '/dev/video*'}, function(result) {
        $scope.video_devices = result.paths.sort();
    });

    $scope.$watch('selected_video_device', function(new_device) {
        if (new_device) {
            $rosparam.set('/camera/video_device', new_device);
        }
    });
})
.controller('waypoints', function ($scope, $timeout, Waypoints) {
  $scope.waypoints = [];
  $scope.come_here = function() {
      navigator.geolocation.getCurrentPosition(function(location){
          var waypoints = {
              waypoints: [{
              'latitude': location.coords.latitude,
              'longitude': location.coords.longitude,
              'altitude': 5.0
          }]};
          Waypoints.set(waypoints);
      });
  };
  Waypoints.subscribe(function(waypoints) {
      $scope.waypoints = waypoints.waypoints;
      // XXX: This shouldn't be necessary
      $scope.$apply();
  });
})
.controller('rcstate', function ($scope, RCState) {
  $scope.data = {'channel': []};
  RCState.subscribe(function(rcstate) {
      $scope.data = rcstate;
  });
})
.controller('imu', function ($scope, IMU) {
  $scope.data = {
      'gyro': {'x': 0, 'y': 0, 'z': 0},
      'accel': {'x': 0, 'y': 0, 'z': 0},
      'mag': {'x': 0, 'y': 0, 'z': 0}
  };
  IMU.subscribe(function(data) {
      $scope.data = data;
      if ($('#accel_z_chart').length > 0) {
        var series = $('#accel_z_chart').highcharts().series[0];
        var x = (new Date()).getTime();
        series.addPoint([x, data.accel.z], true, true);
      }
  });
})
.controller('status', function ($scope, $timeout, Status) {
  $scope.data = {armed: false};
  $scope.arm = function() {
      Status.set({
          armed: true
      });
  };
  $scope.disarm = function() {
      Status.set({
          armed: false
      });
  };
  Status.subscribe(function(status) {
      $scope.data = status;
  });
})
.controller('position', function ($scope, $timeout, Position, Waypoints) {
  $scope.waypoint_data = {'waypoints': []};
  // If we don't have an internet connection, Google Maps won't have loaded
  if (typeof(google) != 'undefined' && document.getElementById('map-canvas')) {
    $scope.map = new GMaps({
        div: '#map-canvas',
        lat: 37.77,
        lng: -122.4,
        zoom: 17,
        mapType: 'satellite'
    });

    $scope.map.addMarker({
        lat: 37.77,
        lng: -122.4,
        title: 'Multicopter',
        icon: '/static/red-marker.png'
    });
    $scope.marker = $scope.map.markers[0]
    $scope.map.addMarker({
        lat: 0,
        lng: 0,
        title: 'Waypoint',
        icon: '/static/green-marker.png'
    });
    $scope.waypoint = $scope.map.markers[1];

    $scope.map.setContextMenu({
        control: 'map',
        options: [
            {
                title: 'Set Waypoint',
                name: 'set_waypoint',
                action: function(e) {
                    var waypoints = {
                        waypoints: [{
                        'latitude': e.latLng.lat(),
                        'longitude': e.latLng.lng(),
                        'altitude': 5.0
                    }]};
                    Waypoints.set(waypoints);
                },
            }
        ]
    });
  }

  // GMaps doesn't have touch support, so add our own long press detection
  google.maps.event.addListener($scope.map.map, 'mousedown', function(e) {
      $scope.longpress_promise = $timeout(function() {
        $scope.map.buildContextMenu('map', e)
      }, 1000);
  });
  google.maps.event.addListener($scope.map.map, 'mouseup', function(e) {
      $timeout.cancel($scope.longpress_promise);
  });
  google.maps.event.addListener($scope.map.map, 'dragstart', function(e) {
      $timeout.cancel($scope.longpress_promise);
  });
  
  Waypoints.subscribe(function(data) {
      $scope.waypoint_data = data;
      if (data.waypoints.length > 0) {
          var lat = data.waypoints[0].latitude;
          var lng = data.waypoints[0].longitude;
          var pos = new google.maps.LatLng(lat, lng);
          $scope.waypoint.setPosition(pos);
      }
  });

  Position.subscribe(function(position) {
      $scope.data = position;
      if (typeof($scope.map) != 'undefined') {
        // XXX: This should be moved
        var pos = new google.maps.LatLng(position.latitude,
                                        position.longitude);
        $scope.marker.setPosition(pos);
        $scope.map.setCenter(position.latitude, position.longitude);
      }
  });
})
.controller('attitude', function ($scope, Attitude) {
  var compass = document.getElementById("compass");
  var attitude_svg = document.getElementById("attitude_svg");
  var needle = null;
  var needle_translate = null;
  var roll_gauge = null;
  var roll_gauge_translate = null;
  var roll_needle = null;
  compass.addEventListener('load', function() {
      needle = compass.getSVGDocument().getElementById("needle");
      needle_translate = needle.getAttribute("transform");
  });
  attitude_svg.addEventListener('load', function() {
      roll_needle = attitude_svg.getSVGDocument().getElementById("layer2");
      roll_gauge = attitude_svg.getSVGDocument().getElementById("layer5");
      roll_gauge_translate = roll_gauge.getAttribute("transform");
  });
  $scope.data = {'roll': 0, 'pitch': 0, 'yaw': 0};

  Attitude.subscribe(function(attitude) {
      if (needle != null) {
          var x = needle.getBBox().width / 2.0;
          var y = needle.getBBox().height / 2.0;
          var yaw = attitude.yaw * 180 / Math.PI;
          needle.setAttribute("transform", 
              "rotate(" + -yaw + " " + x + " " + y + ") "
              + needle_translate);
      }

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

      $scope.data = attitude;
  });
})
.controller('graphs', function($scope) {
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
        tickPixelInterval: 150
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
        data: (function() {
            var data = [],
                time = (new Date()).getTime(),
                i;

            for (i = -19; i <= 0; i++) {
                data.push({
                    x: time + i * 1000,
                    y: 0
                });
            }
            return data;
        })()
    }]
  });
})
.controller('camera', function($scope, $timeout, Media, Camera) {
  $scope.media = {'objs': []};

  (function tick() {
      Media.get({}, function(data) {
          if (data.objs.length != $scope.media.objs.length) {
            $scope.media = data;
          }
          $timeout(tick, 1000);
      });
  })();

  $scope.server_name = window.location.hostname;
  $scope.take_picture = Camera.take_picture;

  $scope.recording = false;
  $scope.toggle_recording = function() {
    if ($scope.recording) {
      Camera.stop_recording();
    } else {
      Camera.start_recording();
    }
    $scope.recording = !$scope.recording;
  };
})
.directive('activeClass', function($location) {
    return {
        restrict: 'A',
        link: function(scope, element, attrs, controller) {
            var css = attrs.activeClass;
            var a = element.find('a');
            var href = a.attr('href');
            href = href.substring(1);
            scope.location = $location;
            scope.$watch('location.path()', function(path) {
                if (href === path) {
                    element.addClass(css);
                } else {
                    element.removeClass(css);
                }
            });
        }
    };
});
