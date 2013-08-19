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
import rospy
import rospilot.msg
import px_comm.msg
import geometry_msgs.msg
import std_srvs.srv
import tf
import numpy
import math
import cv2
import threading
from optparse import OptionParser
from rospilot.pid import *


class OpticalFlowNode:
    def __init__(self, video_device_num):
        self.ground_distance = 0
        self.roll = 0
        self.pitch = 0
        self.lock = threading.Lock()

        rospy.init_node("rospilot_odometry")
        rospy.Subscriber("attitude", rospilot.msg.Attitude,
                         self.handle_attitude)
        rospy.Subscriber("px4flow/opt_flow", px_comm.msg.OpticalFlow,
                         self.handle_optical_flow)
        self.optical_flow = rospy.Publisher('optical_flow',
                                            rospilot.msg.OpticalFlow)
        self.video_device_num = int(video_device_num)
        self.camera = cv2.VideoCapture(self.video_device_num)
        self.resolution_x = 640
        self.resolution_y = 480
        # TODO: measure the field of view
        self.field_of_view_x = 75 * math.pi / 180.0
        self.field_of_view_y = 75 * math.pi / 180.0
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, self.resolution_x)
        self.camera.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, self.resolution_y)
        self.last_frame = None
        self.last_roll = 0
        self.last_pitch = 0

    def handle_attitude(self, message):
        with self.lock:
            self.roll = message.roll
            self.pitch = message.pitch

    def handle_optical_flow(self, message):
        with self.lock:
            self.ground_distance = message.ground_distance

    def calc_scaler(self, resolution, fov):
        return 2.0 * math.tan(fov / 2.0) / float(resolution)

    def run(self):
        max_features = 200
        quality_level = 0.05
        min_dist = 1
        while not rospy.is_shutdown():
            read_status, orig_frame = self.camera.read()
            if not read_status:
                rospy.logerr("Failed to read from video device %d. "
                             "Check that the device is connected, "
                             "and that you have permission to read from it.",
                             self.video_device_num)
                break
            # need to convert to greyscale before finding features
            frame = cv2.cvtColor(orig_frame, cv2.COLOR_BGR2GRAY)
            if self.last_frame is None:
                self.last_frame = frame

            features = cv2.goodFeaturesToTrack(self.last_frame, max_features,
                                               quality_level, min_dist)
            new_features = []
            status = []
            errs = []
            if features is not None and len(features) > 1:
                new_features, status, errs = \
                        cv2.calcOpticalFlowPyrLK(self.last_frame, frame, features)
            self.last_frame = frame

            with self.lock:
                # roll/pitch compensation
                droll = self.roll - self.last_roll
                dpitch = self.pitch - self.last_pitch
                # TODO: Add a lock, this and the handlers aren't threadsafe
                self.last_roll = self.roll
                self.last_pitch = self.pitch
                # Assume camera matches body frame (+x toward back and +y to left)
                expected_dx = dpitch*self.resolution_x / float(self.field_of_view_x)
                expected_dy = -droll*self.resolution_y / float(self.field_of_view_y)
                distance = self.ground_distance

            if droll > self.field_of_view_y / 2.0 or \
                    dpitch > self.field_of_view_x / 2.0 or \
                    status is None or len(status) < 10:
                # Too much roll or pitch for reading to be reliable
                continue

            deltas = []
            for found, old_feature, new_feature, err in \
                    zip(status, features, new_features, errs):
                if not found:
                    continue
                old_feature = old_feature[0]
                new_feature = new_feature[0]
                dx = new_feature[0] - old_feature[0]
                dy = new_feature[1] - old_feature[1]
                dx -= expected_dx
                dy -= expected_dy
                # Reverse these, since we want to know which way we moved,
                # not which way the feature moved
                dx *= -1.0
                dy *= -1.0
                dx *= self.calc_scaler(self.resolution_x, self.field_of_view_x)
                dx *= distance
                dy *= self.calc_scaler(self.resolution_y, self.field_of_view_y)
                dy *= distance
                deltas.append((math.sqrt(dx**2 + dy**2), dx, dy))

            deltas.sort()
            # Remove the top and bottom 5%
            l = len(deltas)
            deltas = deltas[int(l*0.05):int(l*0.95)]
            dxs = map(lambda x: x[1], deltas)
            dys = map(lambda x: x[2], deltas)

            message = rospilot.msg.OpticalFlow()
            message.x = numpy.mean(dxs)
            message.y = numpy.mean(dys)
            message.variance_x = numpy.var(dxs)
            message.variance_y = numpy.var(dys)
            self.optical_flow.publish(message)

if __name__ == "__main__":
    parser = OptionParser("rospilot.py <options>")
    parser.add_option("--video_device_num", dest="video_device_num",
            type='int', help="device number", default=0)
    (opts, args) = parser.parse_args()
    node = OpticalFlowNode(video_device_num=opts.video_device_num)
    node.run()










