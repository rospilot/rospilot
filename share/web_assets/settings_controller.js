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
.controller('settings', function ($scope, $rosparam, Camera) {
    $scope.detector_enabled = false;
    $rosparam.get('/rospilot/camera/detector_enabled',
        function(value) {
            $scope.detector_enabled = value;
            $scope.$apply();
        }
    );
    $scope.resolutions = [];
    $scope.selected_resolution = '';
    Camera.resolutions.subscribe(function(resolutions) {
        var options = $.map(resolutions.resolutions, function(value) {
            return value.width + 'x' + value.height;
        });
        $scope.resolutions = options;
        $scope.$apply();
    });
    $rosparam.get('/rospilot/camera/image_width',
        function(width) {
            $rosparam.get('/rospilot/camera/image_height',
                function(height) {
                    $scope.selected_resolution = width + 'x' + height;
                    $scope.$apply();
                }
            );
        }
    );
    $scope.$watch('selected_resolution', function(new_resolution) {
        if (new_resolution) {
            var parts = new_resolution.split('x');
            $rosparam.set('/rospilot/camera/image_width', parseInt(parts[0]));
            $rosparam.set('/rospilot/camera/image_height', parseInt(parts[1]));
        }
    });

    $scope.$watch('detector_enabled', function(value) {
        $rosparam.set('/rospilot/camera/detector_enabled', value);
    });
});
