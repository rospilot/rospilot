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
class RCStateComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rcstate',
            template: '<div>RC Channels: {{channels | async}}</div>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(Copter)
    {
        this.channels = Copter.getRCChannels();
    }
}

class WaypointComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'rospilotwaypoint',
            template: "<div>Lat: {{latitude | async | number:'1.1-5'}}, Lng: {{longitude | async | number:'1.1-5'}}, Alt: {{altitude | async | number:'1.1-1'}}m</div>"
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(Copter)
    {
        this.latitude = Copter.getWaypoint()
            .map(waypoint => waypoint.latitude);
        this.longitude = Copter.getWaypoint()
            .map(waypoint => waypoint.longitude);
        this.altitude = Copter.getWaypoint()
            .map(waypoint => waypoint.altitude);
    }
}

class GlobalPositionComponent
{
    static get annotations()
    {
        return [new ng.core.Component({
            selector: 'globalposition',
            template: '<div>Lat: {{latitude | async}}, Lng: {{longitude | async}}</div>'
        })];
    }

    static get parameters()
    {
        return [Copter];
    }

    constructor(Copter)
    {
        this.latitude = Copter.getGlobalPosition()
            .map(position => position.latitude);
        this.longitude = Copter.getGlobalPosition()
            .map(position => position.longitude);
    }
}
