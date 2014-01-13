angular.module('rospilot', ['ngRoute', 'ngResource'])
.config(function($routeProvider) {
    $routeProvider
    .when("/graphs", {templateUrl:'static/graphs.html', controller:'graphs'})
    .when("/flight_control", {templateUrl: 'static/flight_control.html', controller:'position'})
    .otherwise({redirectTo:"/flight_control"});
})
.factory('Status', function ($resource) {
      return $resource('api/status');
  })
.factory('PositionEstimate', function ($resource) {
      return $resource('api/position_estimate');
  })
.factory('Position', function ($resource) {
      return $resource('api/position');
  })
.factory('Attitude', function ($resource) {
      return $resource('api/attitude');
  })
.factory('RCState', function ($resource) {
      return $resource('api/rcstate');
  })
.factory('IMU', function ($resource) {
      return $resource('api/imu');
  })
.factory('Waypoints', function ($resource) {
      return $resource('api/waypoints');
  })
.factory('OpticalFlow', function ($resource) {
      return $resource('api/optical_flow');
  })
.controller('rospilot', function ($scope, $route) {
    $scope.$route = $route;
})
.controller('waypoints', function ($scope, $timeout, Waypoints) {
  $scope.data = {'waypoints': []};
  $scope.come_here = function() {
      navigator.geolocation.getCurrentPosition(function(location){
          $scope.data.waypoints = [{
              'latitude': location.coords.latitude,
              'longitude': location.coords.longitude,
              'altitude': 5.0
          }];
          $scope.data.$save();
      });
  };
  (function tick() {
      Waypoints.get({}, function(waypoints) {
          $scope.data = waypoints;
          $timeout(tick, 1000);
      });
  })();
})
.controller('rcstate', function ($scope, $timeout, RCState) {
  $scope.data = {'channel': []};
  (function tick() {
      RCState.get({}, function(rcstate) {
          $scope.data = rcstate;
          $timeout(tick, 1000);
      });
  })();
})
.controller('imu', function ($scope, $timeout, IMU) {
  $scope.data = {
      'gyro': {'x': 0, 'y': 0, 'z': 0},
      'accel': {'x': 0, 'y': 0, 'z': 0},
      'mag': {'x': 0, 'y': 0, 'z': 0}
  };
  (function tick() {
      IMU.get({}, function(data) {
          $scope.data = data;
          if ($('#accel_z_chart').length > 0) {
            var series = $('#accel_z_chart').highcharts().series[0];
            var x = (new Date()).getTime();
            series.addPoint([x, data.accel.z], true, true);
          }
          $timeout(tick, 1000);
      });
  })();
})
.controller('optical_flow', function ($scope, $timeout, OpticalFlow) {
  $scope.data = {'x': 0, 'y': 0, 'quality': 0};
  $scope.reset = function() {
      $scope.data.x = 0;
      $scope.data.y = 0;
      $scope.data.$save();
  };
  (function tick() {
      OpticalFlow.get({}, function(data) {
          $scope.data = data;
          $timeout(tick, 1000);
      });
  })();
})
.controller('status', function ($scope, $timeout, Status) {
  $scope.arm = function() {
      $scope.data.armed = true;
      $scope.data.$save();
  };
  $scope.disarm = function() {
      $scope.data.armed = false;
      $scope.data.$save();
  };
  (function tick() {
      Status.get({}, function(status) {
          $scope.data = status;
          $timeout(tick, 1000);
      });
  })();
})
.controller('position', function ($scope, $timeout, Position, PositionEstimate, Waypoints) {
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
                    $scope.waypoint_data.waypoints = [{
                        'latitude': e.latLng.lat(),
                        'longitude': e.latLng.lng(),
                        'altitude': 5.0
                    }];
                    $scope.waypoint_data.$save();
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
  
  (function tick3() {
      Waypoints.get({}, function(data) {
          $scope.waypoint_data = data;
          if (data.waypoints.length > 0) {
              var lat = data.waypoints[0].latitude;
              var lng = data.waypoints[0].longitude;
              var pos = new google.maps.LatLng(lat, lng);
              $scope.waypoint.setPosition(pos);
          }
          $timeout(tick3, 1000);
      });
  })();

  (function tick2() {
      PositionEstimate.get({}, function(estimate) {
          $scope.estimate = estimate;
          if ($('#position_z_chart').length > 0) {
            var series = $('#position_z_chart').highcharts().series[0];
            var x = (new Date()).getTime();
            series.addPoint([x, estimate.position.z], true, true);
          }
          $timeout(tick2, 100);
      });
  })();

  (function tick() {
      Position.get({}, function(position) {
          $scope.data = position;
          if (typeof($scope.map) != 'undefined') {
            // XXX: This should be moved
            var pos = new google.maps.LatLng(position.latitude,
                                            position.longitude);
            $scope.marker.setPosition(pos);
            $scope.map.setCenter(position.latitude, position.longitude);
          }
          if ($('#position_z_chart').length > 0) {
            var series = $('#position_z_chart').highcharts().series[1];
            var x = (new Date()).getTime();
            series.addPoint([x, position.ground_distance], true, true);
          }
          $timeout(tick, 100);
      });
  })();
})
.controller('attitude', function ($scope, $timeout, Attitude) {
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

  (function tick() {
      Attitude.get({}, function(attitude) {
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
          $timeout(tick, 1000);
      });
  })();
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
  $('#position_z_chart').highcharts({
    chart: {
        type: 'spline',
        animation: false,
        marginRight: 10,
    },
    title: {
        text: 'Altitude'
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
    series: [
    {
        name: 'estimate',
        data: (function() {
            var data = [],
                time = (new Date()).getTime(),
                i;

            for (i = -99; i <= 0; i++) {
                data.push({
                    x: time + i * 1000,
                    y: 0
                });
            }
            return data;
        })()
    },
    {
        name: 'data',
        color: 'red',
        data: (function() {
            var data = [],
                time = (new Date()).getTime(),
                i;

            for (i = -99; i <= 0; i++) {
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
