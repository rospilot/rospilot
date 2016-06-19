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
.controller('camera', function($scope, VideoStream, OnboardComputer) {
  var renderer = PIXI.autoDetectRenderer(640, 480, {transparent: true});
  VideoStream.getResolution().subscribe(resolution => {
      renderer.resize(resolution.width, resolution.height);
      document.getElementById('video').style.width = resolution.width + "px";
      document.getElementById('video').style.height = resolution.height + "px";
  });

  document.querySelector('#video').appendChild(renderer.view);
  renderer.view.style.zIndex = "2";

  // create the root of the scene graph
  var stage = new PIXI.Container();
  var textObjs = new Map();
  OnboardComputer.getVisionTargets().subscribe(function(message) {
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

  document.querySelector('#video').appendChild(VideoStream.getCanvas());
  VideoStream.getCanvas().style.zIndex = "1";
});
