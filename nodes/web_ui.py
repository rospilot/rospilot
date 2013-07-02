#!/usr/bin/env python
'''
Copyright 2012 Christopher Berner

This file is part of Rospilot.

Rospilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Rospilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Rospilot.  If not, see <http://www.gnu.org/licenses/>.
'''

import roslib; roslib.load_manifest('rospilot')
import rospilot
import rospy
from pymavlink import mavutil
import rospilot.msg
import json

from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer

PORT_NUMBER = 8085

armed = None
gps = None

#This class will handles any incoming request from
#the browser 
class HttpHandler(BaseHTTPRequestHandler):
    
    #Handler for the GET requests
    def do_GET(self):
        js = """
    <script src="https://maps.googleapis.com/maps/api/js?v=3.exp&sensor=false"></script>
    <script src="https://ajax.googleapis.com/ajax/libs/angularjs/1.0.7/angular.min.js"></script>
    <script src="https://ajax.googleapis.com/ajax/libs/angularjs/1.0.7/angular-resource.min.js"></script>
    <script>

    var global_marker = false;
    var global_map = false;

var app = angular.module('rospilot', ['ngResource'])
.factory('Position', function ($resource) {
        return $resource('api/position');
    })
.controller('position', function ($scope, $timeout, Position) {
    $scope.data = {'latitude': 1, 'longitude': 1};
    (function tick() {
        Position.get({}, function(position) {
            $scope.data = position;
            // XXX: This should be moved
            var pos = new google.maps.LatLng(position.latitude,
                                             position.longitude);
            if (global_marker) {
                global_marker.setPosition(pos);
            }
            if (global_map) {
                global_map.setCenter(pos);
            }
            $timeout(tick, 1000);
        });
    })();
});

function initialize() {
  var myLatlng = new google.maps.LatLng(1,1);
  var mapOptions = {
    zoom: 18,
    center: myLatlng,
    mapTypeId: google.maps.MapTypeId.SATELLITE
  }
  global_map = new google.maps.Map(document.getElementById('map-canvas'), mapOptions);

  global_marker = new google.maps.Marker({
      position: myLatlng,
      map: global_map,
      title: 'GPS Map'
  });
}

google.maps.event.addDomListener(window, 'load', initialize);

    </script>
        """
        if 'api/position' in self.path:
            self.send_response(200)
            self.send_header('Content-type','application/json')
            self.end_headers()
            # Send the html message
            self.wfile.write(
                    json.dumps({
                        'latitude': gps.latitude if gps else 1,
                        'longitude': gps.longitude if gps else 1}))

        else:
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            # Send the html message
            self.wfile.write(
                    "<html ng-app=rospilot>"
                    "<head>"
                    + js + 
                    "</head>"
                    "<body>"
                    "<div id=\"map-canvas\" style='height:400px;width:500px;'></div>"
                    "<div ng-controller='position'>{{data.latitude}}, {{data.longitude}}</div>"
                    "<h2" + (" style='color:red;'" if armed else "") + ">" 
                    + ("ARMED" if armed else "DISARMED") + 
                    "</h2>"
                    "<a href='/arm'>"
                    + ("disarm" if armed else "arm") + 
                    "</a>"
                    "</body>"
                    "</html>")
            if 'arm' in self.path:
                node.send_arm(not armed)
        return

class WebUiNode:
    def __init__(self):
        self.pub_set_mode = rospy.Publisher('set_mode', rospilot.msg.BasicMode)
        rospy.Subscriber("basic_status", rospilot.msg.BasicMode, self.handle_status)
        rospy.Subscriber("gpsraw", rospilot.msg.GPSRaw, self.handle_gps)
        self.http_server = HTTPServer(('', PORT_NUMBER), HttpHandler)
        self.http_server.timeout = 1.0

    def handle_status(self, data):
        global armed
        armed = data.armed

    def handle_gps(self, data):
        global gps
        gps = data

    def send_arm(self, arm):
        self.pub_set_mode.publish(arm)

    def run(self):
        rospy.init_node('rospilot_webui')
        rospy.loginfo("Web UI is running")
        while not rospy.is_shutdown():
            self.http_server.handle_request()

if __name__ == '__main__':
    node = WebUiNode()
    node.run()
