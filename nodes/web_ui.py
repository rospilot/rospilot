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
import sensor_msgs.msg
import cv_bridge
import cv2
import numpy
from geometry_msgs.msg import Vector3
import tf
import urllib2
from optparse import OptionParser

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
    def media(self):
        paths = os.listdir(self.node.media_path)
        paths = ['/media/' + path for path in paths]
        objs = []
        for path in reversed(sorted(paths)):
            if path.endswith('jpg'):
                objs.append({"type": "image", "url": path})
            else:
                objs.append({"type": "video", "url": path})
        return json.dumps({'objs': objs})

    @cherrypy.expose
    def camera(self, action):
        if cherrypy.request.method == 'GET':
            url = 'http://localhost:8080/snapshot?topic=/camera/image_raw'
            resp = urllib2.urlopen(url)
            cherrypy.response.headers['Content-Type'] = resp.info()['Content-Type']
            return resp.read()
        elif cherrypy.request.method == 'POST':
            if action == 'take_picture':
                name = self.node.take_picture()
                return json.dumps({'url': '/media/' + name})
            elif action == 'record':
                self.node.start_recording()
            elif action == 'stop':
                self.node.stop_recording()
            else:
                return json.dumps({"error": "bad action"})

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
    def waypoints(self):
        if cherrypy.request.method == 'GET':
            waypoints = []
            for waypoint in node.waypoints:
                waypoints.append({'latitude': waypoint.latitude,
                                'longitude': waypoint.longitude,
                                'altitude': waypoint.altitude})
            return json.dumps({'waypoints': waypoints})
        elif cherrypy.request.method == 'POST':
            data = json.loads(cherrypy.request.body.read())
            waypoints = data.get('waypoints', [])
            waypoints = map(lambda x: rospilot.msg.Waypoint(**x), waypoints)
            node.pub_waypoints.publish(waypoints)

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
    def __init__(self, media_path):
        self.reset_odometry = rospy.ServiceProxy('reset_odometry', std_srvs.srv.Empty)
        self.pub_set_mode = rospy.Publisher('set_mode', rospilot.msg.BasicMode)
        self.pub_waypoints = rospy.Publisher('set_waypoints', rospilot.msg.Waypoints)
        self.tf_listener = None
        self.start_record_proxy = rospy.ServiceProxy('start_record', std_srvs.srv.Empty)
        self.stop_record_proxy 	= rospy.ServiceProxy('stop_record',  std_srvs.srv.Empty)
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
        rospy.Subscriber('waypoints',
                rospilot.msg.Waypoints, self.handle_waypoints)
        rospy.Subscriber('camera/image_raw',
                sensor_msgs.msg.Image, self.handle_image)
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
        self.waypoints = []
        self.last_image = None
        self.recording = False
        self.media_path = os.path.expanduser(media_path)
        if not os.path.exists(self.media_path):
            os.makedirs(self.media_path)

        cherrypy.server.socket_port = PORT_NUMBER
        cherrypy.server.socket_host = '0.0.0.0'
        # No autoreloading
        cherrypy.engine.autoreload.unsubscribe()
        conf = {
            '/static': {'tools.staticdir.on': True,
                'tools.staticdir.dir': STATIC_PATH
            },
            '/media': {'tools.staticdir.on': True,
                'tools.staticdir.dir': self.media_path
            }
        }
        index = Index(self)
        index.api = API(self)
        cherrypy.tree.mount(index, config=conf)
        cherrypy.log.screen = False

    def init(self):
        """Called after rospy.init_node()"""
        self.tf_listener = tf.TransformListener()

    def handle_image(self, data):
        with self.lock:
            self.last_image = data

    def handle_attitude(self, data):
        with self.lock:
            self.attitude = data

    def handle_waypoints(self, data):
        with self.lock:
            self.waypoints = data.waypoints

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

    def next_media_id(self):
        return int(round(time.time() * 1000))

    def start_recording(self):
        with self.lock:
            if self.recording:
                rospy.logwarn("Can't start recording. We're already recording")
            self.recording = True
            (self.start_record_proxy)()

    def stop_recording(self):
        with self.lock:
            self.recording = False
            (self.stop_record_proxy)()

    def take_picture(self):
        next_id = self.next_media_id()

        filename = "{0:05}.jpg".format(next_id)
        path = "{0}/{1}".format(self.media_path, filename)

        image = cv_bridge.CvBridge().imgmsg_to_cv(self.last_image,
                                                    desired_encoding="bgr8")
        # cv_bridge returns a cv2.cv.cvmat
        image = numpy.asarray(image)
        cv2.imwrite(path, image)

        return filename

    def run(self):
        rospy.init_node('rospilot_webui')
        self.init()
        rospy.loginfo("Web UI is running")
        cherrypy.engine.start()
        rospy.spin()
        cherrypy.engine.exit()

if __name__ == '__main__':
    parser = OptionParser("web_ui.py <options>")
    parser.add_option(
        "--media_path",
        dest    = "media_path",
        type    = 'string',
        help    = "Directory to store media generated by drone",
        default = "/tmp")
    (opts, args) = parser.parse_args()

    node = WebUiNode(media_path = opts.media_path)
    node.run()
