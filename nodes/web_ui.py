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
import px_comm.msg
import std_srvs.srv
import geometry_msgs.msg
import tf

STATIC_PATH = os.path.dirname(os.path.abspath(__file__))
STATIC_PATH = os.path.join(STATIC_PATH, "../static")

PORT_NUMBER = 8085

class API:
    def __init__(self, node):
        self.node = node

    @cherrypy.expose
    def position(self):
        ground_distance = 0
        if node.distances:
            distances = node.distances[-10:]
            ground_distance = sum(distances) / max(1, len(distances))

        return json.dumps({
            'latitude': node.gps.latitude if node.gps else 1,
            'longitude': node.gps.longitude if node.gps else 1,
            'ground_distance': ground_distance})
    
    @cherrypy.expose
    def optical_flow(self):
        if cherrypy.request.method == 'GET':
            quality = 0
            if node.qualities:
                qualities = node.qualities[-10:]
                quality = sum(qualities) / max(1, len(qualities))
            return json.dumps({
                'x': node.odometry.x if node.odometry else 0,
                'y': node.odometry.y if node.odometry else 0,
                'quality': quality})
        elif cherrypy.request.method == 'POST':
            data = json.loads(cherrypy.request.body.read())
            if data.get('x', 1) == 0 and data.get('y', 1) == 0:
                node.reset_odometry()
    
    @cherrypy.expose
    def attitude(self):
        return json.dumps({
            'roll': node.attitude.roll if node.attitude else 0,
            'pitch': node.attitude.pitch if node.attitude else 0,
            'yaw': node.attitude.yaw if node.attitude else 0})
    
    @cherrypy.expose
    def rcstate(self):
        return json.dumps({
            'channel': node.rcstate.channel if node.rcstate else []})

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
        self.reset_odometry = rospy.ServiceProxy('reset_odometry', std_srvs.srv.Empty) 
        self.pub_set_mode = rospy.Publisher('set_mode', rospilot.msg.BasicMode)
        self.tf_listener = None
        rospy.Subscriber("basic_status", 
                rospilot.msg.BasicMode, self.handle_status)
        rospy.Subscriber("gpsraw", 
                rospilot.msg.GPSRaw, self.handle_gps)
        rospy.Subscriber("attitude", 
                rospilot.msg.Attitude, self.handle_attitude)
        rospy.Subscriber("px4flow/opt_flow", 
                px_comm.msg.OpticalFlow, self.handle_px4flow)
        rospy.Subscriber('odometry', 
                geometry_msgs.msg.Point, self.handle_odometry)
        rospy.Subscriber('op_control', 
                geometry_msgs.msg.Vector3, self.handle_control)
        rospy.Subscriber('rcstate', 
                rospilot.msg.RCState, self.handle_rcstate)
        self.qualities = []
        self.distances = []
        self.odometry = None
        self.control = None
        self.rcstate = None
        self.lock = threading.Lock()
        self.armed = None
        self.gps = None
        self.attitude = None
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

    def init(self):
        """Called after rospy.init_node()"""
        self.tf_listener = tf.TransformListener()

    def handle_attitude(self, data):
        with self.lock:
            self.attitude = data

    def handle_odometry(self, data):
        with self.lock:
            self.odometry = data

    def handle_rcstate(self, data):
        with self.lock:
            self.rcstate = data
    
    def handle_control(self, data):
        with self.lock:
            self.control = data

    def handle_status(self, data):
        with self.lock:
            self.armed = data.armed

    def handle_gps(self, data):
        with self.lock:
            self.gps = data

    def handle_px4flow(self, data):
        with self.lock:
            self.qualities.append(data.quality)
            self.distances.append(data.ground_distance)
            while len(self.qualities) > 100:
                self.qualities.pop(0)
            while len(self.distances) > 100:
                self.distances.pop(0)

    def send_arm(self, arm):
        self.pub_set_mode.publish(arm)

    def run(self):
        rospy.init_node('rospilot_webui')
        self.init()
        rospy.loginfo("Web UI is running")
        cherrypy.engine.start()
        while not rospy.is_shutdown():
            pass
        cherrypy.engine.exit()

if __name__ == '__main__':
    node = WebUiNode()
    node.run()
