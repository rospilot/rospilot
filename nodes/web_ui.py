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
import rospy
from pymavlink import mavutil
import rospilot.msg

from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer

PORT_NUMBER = 8085

armed = None

#This class will handles any incoming request from
#the browser 
class HttpHandler(BaseHTTPRequestHandler):
    
    #Handler for the GET requests
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type','text/html')
        self.send_header('Refresh','1;url=/')
        self.end_headers()
        # Send the html message
        self.wfile.write(
                "<html>"
                "<body>"
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
        self.http_server = HTTPServer(('', PORT_NUMBER), HttpHandler)

    def handle_status(self, data):
        global armed
        armed = data.armed

    def send_arm(self, arm):
        self.pub_set_mode.publish(arm)

    def run(self):
        rospy.init_node('rospilot_webui')
        rospy.loginfo("Running")
        while not rospy.is_shutdown():
            self.http_server.handle_request()
        self.http_server.close()

if __name__ == '__main__':
    node = WebUiNode()
    node.run()
