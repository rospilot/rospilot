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
    .when("/settings", {templateUrl:'static/settings.html'})
    .when("/flight_control", {templateUrl: 'static/flight_control.html'})
    .otherwise({redirectTo:"/flight_control"});
})
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
.factory('Camera', function ($rosservice) {
    return {
        take_picture: $rosservice('/rospilot/camera/capture_image', 'std_srvs/Empty'),
        start_recording: $rosservice('/rospilot/camera/start_record', 'std_srvs/Empty'),
        stop_recording: $rosservice('/rospilot/camera/stop_record', 'std_srvs/Empty')
    };
})
.factory('Media', function ($resource) {
      return $resource('/api/media/:mediaId', {mediaId: '@id'});
})
.factory('Status', function ($rostopic, $rosservice) {
      return {
          subscribe: function(callback) {
              $rostopic('/rospilot/basic_status', 'rospilot/BasicStatus').subscribe(callback);
          },
          set: $rosservice('/rospilot/set_mode', 'rospilot/SetBasicMode')
      };
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
})
.controller('rospilot', function ($scope, $route) {
    $scope.$route = $route;
})
.controller('settings', function ($scope, $rosparam, $rosservice) {
    var shutdownService = $rosservice('/rospilot/shutdown', 'std_srvs/Empty');
    var globService = $rosservice('/rospilot/glob', 'rospilot/Glob');
    $scope.selected_codec = '';
    $scope.codecs = ['h264', 'mjpeg'];
    $scope.selected_video_device = '';
    $scope.video_devices = [];
    $scope.shutdown = shutdownService;
    $rosparam.get('/rospilot/camera/codec',
        function(value) {
            $scope.selected_codec = value;
            $scope.$apply();
        }
    );
    $rosparam.get('/rospilot/camera/video_device',
        function(value) {
            $scope.selected_video_device = value;
            $scope.$apply();
        }
    );
    globService({pattern: '/dev/video*'}, function(result) {
        $scope.video_devices = result.paths.sort();
        $scope.$apply();
    });

    $scope.$watch('selected_video_device', function(new_device) {
        if (new_device) {
            $rosparam.set('/rospilot/camera/video_device', new_device);
        }
    });
    $scope.$watch('selected_codec', function(new_codec) {
        if (new_codec) {
            $rosparam.set('/rospilot/camera/codec', new_codec);
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
  $scope._last_redraw = new Date().getTime();
  IMU.subscribe(function(data) {
      $scope.data = data;
      if ($('#accel_z_chart').length > 0) {
        var series = $('#accel_z_chart').highcharts().series[0];
        var x = (new Date()).getTime();
        var redraw = x - $scope._last_redraw > 500;
        var extremes = series.xAxis.getExtremes();
        // Shift out the data if there's more than 15secs on-screen
        var shift = extremes.dataMax - extremes.dataMin > 15000;
        if (redraw) {
            $scope._last_redraw = x;
        }
        series.addPoint([x, data.accel.z], redraw, shift);
      }
  });
})
.controller('status', function ($scope, $timeout, Status) {
  $scope.data = {armed: false, flight_mode: ''};
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
  $scope.map = L.map('map', {
      crs: L.CRS.EPSG3857,
      center: {lat: 37.77, lng: -122.49},
      zoom: 10,
      contextmenu: true,
      contextmenuWidth: 150,
      contextmenuItems: [
          {
              text: 'Set Waypoint',
              callback: function(e) {
                    var waypoints = {
                        waypoints: [{
                        'latitude': e.latlng.lat,
                        'longitude': e.latlng.lng,
                        'altitude': 5.0
                    }]};
                    Waypoints.set(waypoints);
                },
          }
      ],
  });
  $scope.server_name = window.location.hostname;
  var mapnik_url = 'http://' + $scope.server_name + ':8086/ex/{z}/{x}/{y}.png';

  L.tileLayer(mapnik_url, {
      maxZoom: 18,
  }).addTo($scope.map);

  var copterIcon = L.AwesomeMarkers.icon({
      prefix: 'fa',
      icon: 'plane',
      markerColor: 'red'
  });

  $scope.marker = L.marker([37.77, -122.49], {
      title: 'Multicopter',
      icon: copterIcon
  }).addTo($scope.map);
  
  var waypointIcon = L.AwesomeMarkers.icon({
      prefix: 'fa',
      icon: 'flag',
      markerColor: 'green'
  });
  
  $scope.waypoint_marker = L.marker([37.77, -122.49], {
      title: 'Waypoint',
      icon: waypointIcon
  }).addTo($scope.map);
  
  Waypoints.subscribe(function(data) {
      $scope.waypoint_data = data;
      if (data.waypoints.length > 0) {
          var lat = data.waypoints[0].latitude;
          var lng = data.waypoints[0].longitude;
        // XXX: This should be moved
          $scope.waypoint_marker.setLatLng([lat, lng]);
      }
  });

  Position.subscribe(function(position) {
      $scope.data = position;
      // XXX: This should be moved
      $scope.marker.setLatLng([position.latitude, position.longitude]);
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
})
.controller('camera', function($scope, $timeout, Media, Camera) {
  $scope.media = [];

  (function tick() {
      Media.query(function(data) {
          if (data.length != $scope.media.length) {
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
.directive('ngConfirmClick', [function() {
    return {
        restrict: 'A',
        link: function(scope, element, attrs) {
            element.bind('click', function() {
                var message = attrs.ngConfirmMessage;
                if (!message) {
                    message = "<insert message here>"
                }
                if (message && confirm(message)) {
                    scope.$apply(attrs.ngConfirmClick);
                }
            });
        }
    }
}])
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
