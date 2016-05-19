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
upgradeAdapter.addProvider(RosParam);
upgradeAdapter.addProvider(Copter);
upgradeAdapter.upgradeNg1Provider('ROS');
upgradeAdapter.upgradeNg1Provider('$rostopic');
upgradeAdapter.upgradeNg1Provider('$rosservice');
angular.module('rospilot')
  .service('$rosparam', upgradeAdapter.downgradeNg2Provider(RosParam))
  .directive('rospilotstatus', upgradeAdapter.downgradeNg2Component(StatusComponent))
  .directive('rospilotwaypoint', upgradeAdapter.downgradeNg2Component(WaypointComponent))
  .directive('globalposition', upgradeAdapter.downgradeNg2Component(GlobalPositionComponent))
  .directive('rospilotmap', upgradeAdapter.downgradeNg2Component(MapComponent))
  .directive('rospilotcomehere', upgradeAdapter.downgradeNg2Component(ComeHereComponent))
  .directive('rcstate', upgradeAdapter.downgradeNg2Component(RCStateComponent));
angular.element(document).ready(function() {
    upgradeAdapter.bootstrap(document, ['rospilot']);
});
