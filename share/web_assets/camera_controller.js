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
.factory('Media', function ($resource) {
      return $resource('/api/media/:mediaId', {mediaId: '@id'});
})
.controller('camera', function($scope, $timeout, $http, $rosparam, Media, Camera, VisionTargets) {
  $scope.media = [];
  $scope.fps = 0;

  $scope.destroyed = false;
  $scope.$on("$destroy", function() {$scope.destroyed = true});

  (function tick() {
      Media.query(function(data) {
          if (data.length != $scope.media.length) {
            $scope.media = data;
          }
          if ($scope.destroyed) {
            return;
          }
          $timeout(tick, 1000);
      });
  })();

  $scope.destroyed = false;
  $scope.$on("$destroy", function() {$scope.destroyed = true});

  var server_name = window.location.hostname;

  // Generate a random client id for fetching the stream
  var clientId = Math.floor(Math.random() * 1000 * 1000 * 1000);
  var videoWidth = 640;
  var videoHeight = 480;
  var player = new Player({size: {height: videoHeight, width: videoWidth}});

  var fpsStartTime = new Date().getTime();
  var frameCount = 0;
  var renderer = PIXI.autoDetectRenderer(videoWidth, videoHeight, {transparent: true});
  player.onPictureDecoded = function(data, width, height) {
      if (width != videoWidth || height != videoHeight) {
          renderer.resize(width, height);
          videoWidth = width;
          videoHeight = height;
          document.getElementById('video').style.width = width + "px";
          document.getElementById('video').style.height = height + "px";
      }
      frameCount++;
      var currentTime = new Date().getTime();
      var deltaSeconds = (currentTime - fpsStartTime) / 1000.0
      if (deltaSeconds > 1) {
          $scope.fps = Math.floor(frameCount / deltaSeconds);
          frameCount = 0;
          fpsStartTime = currentTime;
      }
  };

  document.querySelector('#video').appendChild(renderer.view);
  renderer.view.style.zIndex = "2";

  // create the root of the scene graph
  var stage = new PIXI.Container();
  var textObjs = new Map();
  VisionTargets.subscribe(function(message) {
      var targetIds = new Set();
      for (let target of message.targets) {
          targetIds.add(target.id);
          if (!textObjs.has(target.id)) {
              textObjs.set(target.id, new PIXI.Text(target.description, {fill: 'red'}));
              stage.addChild(textObjs.get(target.id));
          }
          var textObj = textObjs.get(target.id);
          textObj.visible = true;
          textObj.x = videoWidth * (target.x + 1) / 2.0;
          textObj.y = videoHeight * (1 - target.y) / 2.0;
      }
      for (let [key, value] of textObjs) {
          if (!targetIds.has(key)) {
              // Hide element rather than removing it. Because adding/removing is more expensive
              value.visible = false;
          }
      }
      renderer.render(stage);
  });

  document.querySelector('#video').appendChild(player.canvas);
  player.canvas.style.zIndex = "1";
  var h264Url = 'http://' + server_name + ':8666/h264/' + clientId;
  
  function nextFrame() {
      $http.get(h264Url, {responseType: 'arraybuffer'})
      .success(function(data) {
        player.decode(new Uint8Array(data));
        if ($scope.destroyed) {
            return;
        }
        $timeout(nextFrame, 1);
      })
      .error(function() {
        if ($scope.destroyed) {
            return;
        }
        $timeout(nextFrame, 1000);
      });
  };
  nextFrame();

  $scope.take_picture = Camera.take_picture;
});
