var app = angular.module('rospilot', ['ngResource'])
.factory('Status', function ($resource) {
      return $resource('api/status');
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
.factory('OpticalFlow', function ($resource) {
      return $resource('api/optical_flow');
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
.controller('position', function ($scope, $timeout, Position) {
  var myLatlng = new google.maps.LatLng(37.77,122.4);
  var mapOptions = {
    zoom: 18,
    center: myLatlng,
    mapTypeId: google.maps.MapTypeId.SATELLITE
  }
  $scope.map = new google.maps.Map(document.getElementById('map-canvas'), mapOptions);

  $scope.marker = new google.maps.Marker({
      position: myLatlng,
      map: $scope.map,
      title: 'GPS Map'
  });

  (function tick() {
      Position.get({}, function(position) {
          $scope.data = position;
          // XXX: This should be moved
          var pos = new google.maps.LatLng(position.latitude,
                                           position.longitude);
          $scope.marker.setPosition(pos);
          $scope.map.setCenter(pos);
          $timeout(tick, 1000);
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
});

