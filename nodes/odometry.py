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
import rospilot.msg
import px_comm.msg
import geometry_msgs.msg
import std_srvs.srv
import tf
import numpy
from rospilot.pid import *

class OdometryNode:
    def __init__(self):
        self.last_time = 0
        self.total_x = 0
        self.total_y = 0
        self.total_z = 0
        self.last_yaw = 0
        self.pid_x = PidController(100, 0, 10)
        self.pid_y = PidController(100, 0, 10)

        rospy.init_node("rospilot_odometry")
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber("attitude", rospilot.msg.Attitude, 
                self.handle_attitude)
        rospy.Subscriber("px4flow/opt_flow", px_comm.msg.OpticalFlow, 
                self.handle_optical_flow)
        self.odometry = rospy.Publisher('odometry', geometry_msgs.msg.Point)
        self.control_pub = rospy.Publisher('op_control', geometry_msgs.msg.Vector3)
        self.rc_control_pub = rospy.Publisher('set_rc', rospilot.msg.RCState)
        rospy.Subscriber('rcstate', rospilot.msg.RCState, self.handle_rcstate)
        rospy.Service('reset_odometry', std_srvs.srv.Empty, self.handle_reset)

    def handle_reset(self, req):
        self.total_x = 0
        self.total_y = 0
        self.pid_x.reset()
        self.pid_y.reset()

    def handle_rcstate(self, message):
        if message.channel[5] > 1700:
            self.total_x = 0
            self.total_y = 0
            self.pid_x.reset()
            self.pid_y.reset()

    def handle_attitude(self, message):
        self.last_yaw = message.yaw

    def handle_optical_flow(self, message):
        if self.last_time == 0:
            self.last_time = message.header.stamp
            return
        if message.quality < 60:
            return
        dt = (message.header.stamp - self.last_time).to_sec()
        dx = dt * message.velocity_x
        dy = dt * message.velocity_y
        dv =  (dx, dy, 0, 0)
        # Convert to body coords
        m = tf.transformations.euler_matrix(0, 3.14159265359, -1.0471975512)
        dv = m.dot(dv)
        # Convert to world coords
        #m = tf.transformations.euler_matrix(0, 0, -self.last_yaw)
        #dv = m.dot(dv)

        self.last_time = message.header.stamp
        self.total_z = message.ground_distance
        self.total_x += dv[0]
        self.total_y += dv[1]
        self.tf_broadcaster.sendTransform(
                (self.total_x, self.total_y, self.total_z),
                tf.transformations.quaternion_from_euler(0, 0, self.last_yaw),
                rospy.Time.now(),
                "body",
                "world")
        self.odometry.publish(self.total_x, self.total_y, self.total_z)
        self.pid_x.update(self.total_x, dt)
        self.pid_y.update(self.total_y, dt)
        self.control_pub.publish(self.pid_x.get_control(),
                self.pid_y.get_control(), 0)
        roll = 1515 - self.pid_x.get_control()
        # Add to y, because pitch PWM gets smaller going forward
        pitch = 1515 + self.pid_y.get_control()
        roll = min(1600, max(1400, roll))
        pitch = min(1600, max(1400, pitch))
        self.rc_control_pub.publish([roll, pitch, 0, 0, 0, 0, 0, 0])

    def run(self):
        self.tf_broadcaster.sendTransform(
                (0, 0, 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "body",
                "world")
        rospy.spin()

if __name__ == "__main__":
    node = OdometryNode()
    node.run()

        








