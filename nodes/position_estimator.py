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
import rospilot.msg
import threading
import pykalman
import time
import px_comm.msg
import numpy as np
from geometry_msgs.msg import Vector3


class NaiveFilter:
    def __init__(self):
        self.pos = 0
        self.vel = 0
        self.acc = 0
        self.last_time = None

    def observe(self, pos, vel, acc):
        if self.last_time is None:
            self.last_time = time.time()
            return
        t = time.time()
        dt = t - self.last_time
        self.last_time = t

        # Enforce one microsecond has always passed
        if dt < 1e-6:
            rospy.logerr("dt < 1e-6")
            return

        self.pos += self.vel * dt
        if pos is not None:
            self.pos = pos
        self.vel += self.acc * dt
        if vel is not None:
            self.vel = vel
        if acc is not None:
            self.acc = acc

    def get_pos(self):
        return self.pos

    def get_vel(self):
        return self.vel

    def get_acc(self):
        return self.acc

    def get(self):
        return [self.pos, self.vel, self.acc]


class PositionFilter:
    def __init__(self, sensor_variance):
        self.kalman = pykalman.KalmanFilter(
                transition_covariance=0.000001*np.eye(3), n_dim_obs=1)
        self.state = [0, 0, 0]
        self.sensor_variance = sensor_variance
        self.covariance = sensor_variance*np.eye(3)
        self.last_time = None

    def observe(self, pos):
        if self.last_time is None:
            self.last_time = time.time()
            return
        t = time.time()
        dt = t - self.last_time
        self.last_time = t

        # Enforce one microsecond has always passed
        if dt < 1e-6:
            rospy.logerr("dt < 1e-6")
            return

        self.state, self.covariance = self.kalman.filter_update(
                self.state, self.covariance,
                [pos],
                transition_matrix=np.array([
                    [1, dt, 0],
                    [0, 1, dt],
                    [0, 0, 0]]),
                observation_matrix=np.array([
                    [1, 0, 0],
                    ]),
                observation_covariance=np.array(self.sensor_variance*np.eye(1)))

    def get_pos(self):
        return self.state[0]

    def get_vel(self):
        return self.state[1]

    def get_acc(self):
        return self.state[2]

    def get(self):
        return self.state


class PositionEstimatorNode:
    def __init__(self):
        rospy.init_node('rospilot_position_estimator')
        self.pub_position_estimate = rospy.Publisher('position_estimate',
                                                     rospilot.msg.PositionEstimate)
        rospy.Subscriber("px4flow/opt_flow",
                         px_comm.msg.OpticalFlow, self.handle_px4flow)
        rospy.Subscriber("optical_flow",
                         rospilot.msg.OpticalFlow, self.handle_optical_flow)
        self.lock = threading.Lock()
        self.naive_filter_x = NaiveFilter()
        self.naive_filter_y = NaiveFilter()
        self.pos_filter_z = PositionFilter(sensor_variance=0.0001)
        self.c = 0

    def _publish(self):
        self.pub_position_estimate.publish(
            Vector3(self.naive_filter_x.get_pos(),
                    self.naive_filter_y.get_pos(),
                    self.pos_filter_z.get_pos()),
            Vector3(self.naive_filter_x.get_vel(),
                    self.naive_filter_y.get_vel(),
                    self.pos_filter_z.get_vel()))

    def handle_px4flow(self, data):
        with self.lock:
            self.pos_filter_z.observe(data.ground_distance)
            self._publish()

    def handle_optical_flow(self, data):
        with self.lock:
            self.naive_filter_x.observe(None, data.x, None)
            self.naive_filter_y.observe(None, data.y, None)
            self._publish()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = PositionEstimatorNode()
    node.run()
