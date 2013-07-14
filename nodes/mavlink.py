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

# These must be the first two imports, as they setup sys.path
import roslib; roslib.load_manifest('rospilot')
import rospilot
# 

import rospy
from pymavlink import mavutil
import rospilot.msg
from geometry_msgs.msg import Vector3

from optparse import OptionParser
class MavlinkNode:
    def __init__(self, device, baudrate, export_host, allow_control):
        self.export_conn = None
        self.rate = 10
        if export_host:
            self.export_conn = mavutil.mavlink_connection(
                    "udp:" + export_host, input=False)
        self.conn = mavutil.mavlink_connection(device, baud=baudrate)
        self.pub_attitude = rospy.Publisher('attitude', rospilot.msg.Attitude)
        self.pub_rcstate = rospy.Publisher('rcstate', rospilot.msg.RCState)
        self.pub_gpsraw = rospy.Publisher('gpsraw', rospilot.msg.GPSRaw)
        self.pub_imuraw = rospy.Publisher('imuraw', rospilot.msg.IMURaw)
        self.pub_basic_status = rospy.Publisher('basic_status', rospilot.msg.BasicStatus)
        rospy.Subscriber("set_mode", rospilot.msg.BasicMode, self.handle_set_mode)
        rospy.Subscriber("set_rc", rospilot.msg.RCState, self.handle_set_rc)
        self.allow_control = allow_control.lower() == "true" or allow_control.lower() == "1"
        self.enable_control = False
        # Safety, in case radio has control enabled on start-up
        self.enable_control_has_been_false = False

    def reset_rc_override(self):
        # Send 0 to reset the channel
        self.conn.mav.rc_channels_override_send(
                self.conn.target_system, self.conn.target_component, 
                0, 0, 0, 0, 0, 0, 0 ,0)

    def handle_set_rc(self, message):
        if self.allow_control and self.enable_control and self.enable_control_has_been_false:
            # channel 8 is ignored, since that's the enable control channel
            self.conn.mav.rc_channels_override_send(
                    self.conn.target_system, self.conn.target_component,
                    message.channel[0], message.channel[1], 
                    message.channel[2], message.channel[3],
                    message.channel[4], message.channel[5],
                    message.channel[6], 0)

    def handle_set_mode(self, data):
        # XXX: This code should work, 
        # but the APM doesn't seem to listen to set_mode messages :(
        #See MAV_MODE_FLAG in pymavlink.mavlinkv10
        #self.conn.mav.set_mode_send(self.conn.target_system,
        #        209 if data.armed else 81, 0)

        # So instead we fake the tranmitter signals 
        self.conn.mav.rc_channels_override_send(self.conn.target_system, 
                self.conn.target_component, 0, 0, 
                1000, #throttle to zero
                2000 if data.armed else 1000, #yaw full right to arm, left to disarm
                0, 0, 0, 0)
        rospy.sleep(5)
        self.conn.mav.rc_channels_override_send(self.conn.target_system, 
                self.conn.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

    def run(self):
        rospy.init_node('rospilot_mavlink')
        rospy.loginfo("Waiting for heartbeat")
        self.conn.wait_heartbeat()
        rospy.loginfo("Got heartbeat. Waiting 10secs for APM to be ready")
        rospy.sleep(10)

        self.conn.mav.request_data_stream_send(self.conn.target_system,
                self.conn.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 
                self.rate, 1)
        while not rospy.is_shutdown():
            rospy.sleep(0.001)
            msg = self.conn.recv_match(blocking=False)
            if not msg:
                continue
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                rospy.logwarn("Got bad data")
                continue
            if self.export_conn:
                self.export_conn.mav.send(msg)

            if msg_type == "ATTITUDE":
                self.pub_attitude.publish(msg.roll, msg.pitch, msg.yaw, 
                        msg.rollspeed, msg.pitchspeed, msg.yawspeed)
            elif msg_type == "RC_CHANNELS_RAW":
                self.pub_rcstate.publish([msg.chan1_raw, msg.chan2_raw, 
                    msg.chan3_raw, msg.chan4_raw, msg.chan5_raw, msg.chan6_raw,
                    msg.chan7_raw, msg.chan8_raw]) 
                self.enable_control = msg.chan8_raw > 1700
                if not self.enable_control:
                    self.enable_control_has_been_false = True
                    self.reset_rc_override()
            elif msg_type == "RC_CHANNELS_SCALED":
                pass
            elif msg_type == "HEARTBEAT":
                self.pub_basic_status.publish(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif msg_type == "GPS_RAW_INT":
                self.pub_gpsraw.publish(
                        msg.time_usec, msg.fix_type, 
                        msg.lat / float(10*1000*1000),
                        msg.lon / float(10*1000*1000), 
                        msg.alt / float(1000), msg.satellites_visible)
            elif msg_type == "RAW_IMU":
                self.pub_imuraw.publish(
                        msg.time_usec, 
                        Vector3(msg.xgyro, msg.ygyro, msg.zgyro), 
                        Vector3(msg.xacc, msg.yacc, msg.zacc), 
                        Vector3(msg.xmag, msg.ymag, msg.zmag))

if __name__ == '__main__':
    parser = OptionParser("rospilot.py <options>")
    parser.add_option("--baudrate", dest="baudrate", 
            type='int', help="serial port baud rate", default=115200)
    parser.add_option("--allow-control", dest="allow_control", 
            help="allow sending control signals to autopilot", default="false")
    parser.add_option("--device", dest="device", 
            default=None, help="serial device")
    parser.add_option("--udp-export", dest="export_host", 
            default=None, help="UDP host/port to send copy of MAVLink data to")
    (opts, args) = parser.parse_args()

    node = MavlinkNode(device=opts.device, baudrate=opts.baudrate, 
            export_host=opts.export_host, allow_control=opts.allow_control)
    node.run()
