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
import cherrypy
import threading
import os

STATIC_PATH = os.path.dirname(os.path.abspath(__file__))
STATIC_PATH = os.path.join(STATIC_PATH, "../static")

PORT_NUMBER = 8085

class API:
    def __init__(self, node):
        self.node = node

    @cherrypy.expose
    def position(self):
        return json.dumps({
            'latitude': node.gps.latitude if node.gps else 1,
            'longitude': node.gps.longitude if node.gps else 1})

    @cherrypy.expose
    def status(self):
        if cherrypy.request.method == 'GET':
            return json.dumps({'armed': node.armed})
        elif cherrypy.request.method == 'POST':
            data = json.loads(cherrypy.request.body.read())
            node.send_arm(data['armed'])

class Index:
    def __init__(self, node):
        self.node = node

    @cherrypy.expose
    def index(self):
        return open(os.path.join(STATIC_PATH, "index.html"))

class WebUiNode:
    def __init__(self):
        self.pub_set_mode = rospy.Publisher('set_mode', rospilot.msg.BasicMode)
        rospy.Subscriber("basic_status", rospilot.msg.BasicMode, self.handle_status)
        rospy.Subscriber("gpsraw", rospilot.msg.GPSRaw, self.handle_gps)
        self.lock = threading.Lock()
        self.armed = None
        self.gps = None
        cherrypy.server.socket_port = PORT_NUMBER
        # No autoreloading
        cherrypy.engine.autoreload.unsubscribe()
        conf = {
            '/static': {'tools.staticdir.on': True,
                'tools.staticdir.dir': STATIC_PATH
            }
        }
        index = Index(self)
        index.api = API(self)
        cherrypy.tree.mount(index, config=conf)

    def handle_status(self, data):
        with self.lock:
            self.armed = data.armed

    def handle_gps(self, data):
        with self.lock:
            self.gps = data

    def send_arm(self, arm):
        self.pub_set_mode.publish(arm)

    def run(self):
        rospy.init_node('rospilot_webui')
        rospy.loginfo("Web UI is running")
        cherrypy.engine.start()
        while not rospy.is_shutdown():
            pass
        cherrypy.engine.exit()

if __name__ == '__main__':
    node = WebUiNode()
    node.run()
