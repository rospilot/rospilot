#!/usr/bin/env python

'''
Copyright 2012 the original author or authors.
See the NOTICE file distributed with this work for additional
information regarding copyright ownership.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
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
from geometry_msgs.msg import Vector3
import tf

STATIC_PATH = os.path.dirname(os.path.abspath(__file__))
STATIC_PATH = os.path.join(STATIC_PATH, "../static")

PORT_NUMBER = 8085


def _vector3_to_dict(vec):
    return {'x': vec.x, 'y': vec.y, 'z': vec.z}


class API:
    def __init__(self, node):
        self.node = node

    @cherrypy.expose
    def imu(self):
        gyro = _vector3_to_dict(node.imu.gyro if node.imu else Vector3())
        accel = _vector3_to_dict(node.imu.accel if node.imu else Vector3())
        mag = _vector3_to_dict(node.imu.mag if node.imu else Vector3())
        return json.dumps({
            'gyro': gyro,
            'accel': accel,
            'mag': mag})

    @cherrypy.expose
    def position_estimate(self):
        position = _vector3_to_dict(node.position_estimate.position if
                                    node.position_estimate else Vector3())
        velocity = _vector3_to_dict(node.position_estimate.velocity if
                                    node.position_estimate else Vector3())
        return json.dumps({'position': position, 'velocity': velocity})

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
        rospy.Subscriber("imuraw",
                rospilot.msg.IMURaw, self.handle_imu)
        rospy.Subscriber("position_estimate",
                rospilot.msg.PositionEstimate, self.handle_position_estimate)
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
        self.imu = None
        self.position_estimate = None
        self.attitude = None
        cherrypy.server.socket_port = PORT_NUMBER
        cherrypy.server.socket_host = '0.0.0.0'
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

    def handle_position_estimate(self, data):
        with self.lock:
            self.position_estimate = data

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

    def handle_imu(self, data):
        with self.lock:
            self.imu = data

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
        rospy.spin()
        cherrypy.engine.exit()

if __name__ == '__main__':
    node = WebUiNode()
    node.run()
