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
.controller('status', function ($scope, $timeout, Status) {
  $scope.arm = function() {
      $scope.data.armed = true;
      $scope.data.$save()
  };
  $scope.disarm = function() {
      $scope.data.armed = false;
      $scope.data.$save()
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
  var needle = null;
  var translate = null;
  compass.addEventListener('load', function() {
      needle = compass.getSVGDocument().getElementById("needle");
      translate = needle.getAttribute("transform");
  });

  (function tick() {
      Attitude.get({}, function(attitude) {
          if (translate != null) {
              var x = needle.getBBox().width / 2.0;
              var y = needle.getBBox().height / 2.0;
              var yaw = attitude.yaw * 180 / Math.PI;
              needle.setAttribute("transform", 
                  "rotate(" + -yaw + " " + x + " " + y + ") "
                  + translate);
          }
          $timeout(tick, 1000);
      });
  })();
});

