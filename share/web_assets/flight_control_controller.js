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
angular.module('rospilot')
.controller('imu', function ($scope, IMU) {
  $scope.data = {
      'gyro': {'x': 0, 'y': 0, 'z': 0},
      'accel': {'x': 0, 'y': 0, 'z': 0},
      'mag': {'x': 0, 'y': 0, 'z': 0}
  };
  $scope._last_redraw = new Date().getTime();
  IMU.subscribe(function(data) {
      $scope.data = data;
      $scope.$apply();
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
      $scope.$apply();
  });
});
