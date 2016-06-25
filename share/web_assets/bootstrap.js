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
const routes = [
{path: 'flight_control', component: FlightControlPage},
{path: 'graphs', component: GraphsPage},
{path: 'settings', component: SettingsPage},
{path: 'camera', component: CameraPage},
{path: '', redirectTo: 'flight_control', terminal: true}
];

const APP_ROUTER_PROVIDERS = [
  ng.router.provideRouter(routes)
];

ng.platformBrowserDynamic.bootstrap(RospilotApp, [ng.http.HTTP_PROVIDERS, APP_ROUTER_PROVIDERS,
        // Use # for routing links
        {provide: ng.common.LocationStrategy, useClass: ng.common.HashLocationStrategy},
        RosTopic, RosService, RosParam, RosLib, Copter, Camera, OnboardComputer, VideoStream])
    .catch(err => console.error(err));
