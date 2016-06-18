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
const upgradeAdapter = new ng.upgrade.UpgradeAdapter();
upgradeAdapter.addProvider(RosLib);
upgradeAdapter.addProvider(RosParam);
upgradeAdapter.addProvider(RosService);
upgradeAdapter.addProvider(Copter);
upgradeAdapter.addProvider(OnboardComputer);
upgradeAdapter.addProvider(Camera);
upgradeAdapter.addProvider(ng.http.HTTP_PROVIDERS);
upgradeAdapter.upgradeNg1Provider('$rostopic');
angular.module('rospilot')
  .service('ROS', upgradeAdapter.downgradeNg2Provider(RosLib))
  .service('$rosparam', upgradeAdapter.downgradeNg2Provider(RosParam))
  .directive('videodevices', upgradeAdapter.downgradeNg2Component(VideoDevicesComponent))
  .directive('cameraresolutions', upgradeAdapter.downgradeNg2Component(CameraResolutionsComponent))
  .directive('shutdownbutton', upgradeAdapter.downgradeNg2Component(ShutdownComponent))
  .directive('rospilotmedialist', upgradeAdapter.downgradeNg2Component(MediaListComponent))
  .directive('takepicturebutton', upgradeAdapter.downgradeNg2Component(TakePictureButton))
  .directive('rospilotrecordingbutton', upgradeAdapter.downgradeNg2Component(RecordingButton))
  .directive('rospilotattitude', upgradeAdapter.downgradeNg2Component(AttitudeComponent))
  .directive('rospilotrollguage', upgradeAdapter.downgradeNg2Component(RollGuageComponent))
  .directive('rospilotcompass', upgradeAdapter.downgradeNg2Component(CompassComponent))
  .directive('rospilotaccelerometer', upgradeAdapter.downgradeNg2Component(AccelerometerComponent))
  .directive('rospilotgyroscope', upgradeAdapter.downgradeNg2Component(GyroscopeComponent))
  .directive('rospilotmagnetometer', upgradeAdapter.downgradeNg2Component(MagnetometerComponent))
  .directive('accelerometergraph', upgradeAdapter.downgradeNg2Component(AccelerometerGraphComponent))
  .directive('rospilotstatus', upgradeAdapter.downgradeNg2Component(StatusComponent))
  .directive('rospilotwaypoint', upgradeAdapter.downgradeNg2Component(WaypointComponent))
  .directive('globalposition', upgradeAdapter.downgradeNg2Component(GlobalPositionComponent))
  .directive('rospilotmap', upgradeAdapter.downgradeNg2Component(MapComponent))
  .directive('rospilotcomehere', upgradeAdapter.downgradeNg2Component(ComeHereComponent))
  .directive('rcstate', upgradeAdapter.downgradeNg2Component(RCStateComponent));
angular.element(document).ready(function() {
    upgradeAdapter.bootstrap(document, ['rospilot']);
});
