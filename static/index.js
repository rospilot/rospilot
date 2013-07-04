var app = angular.module('rospilot', ['ngResource'])
.factory('Position', function ($resource) {
      return $resource('api/position');
  })
.controller('position', function ($scope, $timeout, Position) {
  var myLatlng = new google.maps.LatLng(1,1);
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

  $scope.data = {'latitude': 1, 'longitude': 1};
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
});

