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
{path: 'flight_control', component: FlightControlPage, pathMatch: 'full'},
{path: 'settings', component: SettingsPage, pathMatch: 'full'},
{path: 'camera', component: CameraPage, pathMatch: 'full'},
{path: '', redirectTo: 'flight_control', pathMatch: 'full'}
];

class RospilotBootstrapModule
{
    static get annotations()
    {
        return [new ng.core.NgModule({
            declarations: [
                RospilotApp,
                FlightControlPage,
                SettingsPage,
                CameraPage,
                StatusComponent,
                ComeHereComponent,
                FollowMeComponent,
                RollGuageComponent,
                CompassComponent,
                MapComponent,
                WaypointComponent,
                AttitudeComponent,
                GlobalPositionComponent,
                RCStateComponent,
                GyroscopeComponent,
                AccelerometerComponent,
                MagnetometerComponent,
                BatteryComponent,
                AccelerometerClippingComponent,
                VibrationComponent,
                VideoDevicesComponent,
                CameraResolutionsComponent,
                ComputerVisionToggle,
                ShutdownComponent,
                VideoDisplay,
                RecordingButton,
                TakePictureButton,
                FPSDisplay,
                MediaListComponent
            ],
            imports: [
                ng.router.RouterModule.forRoot(routes, {useHash: true}),
                ng.common.CommonModule,
                ng.platformBrowser.BrowserModule,
                ng.forms.FormsModule,
                ng.http.HttpModule
            ],
            providers: [
                RosTopic,
                RosService,
                RosParam,
                RosLib,
                Copter,
                Camera,
                OnboardComputer,
                VideoStream
            ],
            bootstrap: [RospilotApp]
        })];
    }
}

ng.platformBrowserDynamic.platformBrowserDynamic().bootstrapModule(RospilotBootstrapModule)
    .catch(err => console.error(err));
