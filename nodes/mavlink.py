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
import roslib
roslib.load_manifest('rospilot')
import rospilot
#

import rospy
import rospilot.msg
import rospilot.srv
from serial.serialutil import SerialException
from pymavlink import mavutil
from geometry_msgs.msg import Vector3
from optparse import OptionParser
from time import time


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
        self.pub_basic_status = rospy.Publisher('basic_status',
                                                rospilot.msg.BasicStatus)
        self.pub_waypoints = rospy.Publisher('waypoints',
                                             rospilot.msg.Waypoints)
        rospy.Subscriber("set_rc", rospilot.msg.RCState,
                         self.handle_set_rc)
        rospy.Service('set_waypoints',
                      rospilot.srv.SetWaypoints,
                      self.handle_set_waypoints)
        rospy.Service('set_mode',
                      rospilot.srv.SetBasicMode,
                      self.handle_set_mode)
        self.allow_control = allow_control.lower() in ["true", "1"]
        self.enable_control = False
        # Safety, in case radio has control enabled on start-up
        self.enable_control_has_been_false = False

        # Waypoints are read and written using a stateful API
        # this buffer stores the queued writes/partial reads
        self.waypoint_buffer = []
        self.num_waypoints = 0
        self.waypoint_read_in_progress = False
        self.waypoint_write_in_progress = False

    def reset_rc_override(self):
        # Send 0 to reset the channel
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system, self.conn.target_component,
            0, 0, 0, 0, 0, 0, 0, 0)

    def handle_set_waypoints(self, message):
        if self.waypoint_read_in_progress or self.waypoint_write_in_progress:
            rospy.logwarn("Can't write waypoints because a read/write is already in progress")
            return
        self.waypoint_write_in_progress = True
        # XXX: APM seems to overwrite index 0, so insert the first waypoint
        # twice
        self.waypoint_buffer = [message.waypoints[0]] + message.waypoints

        if message.waypoints:
            self.conn.mav.mission_count_send(
                self.conn.target_system,
                self.conn.target_component,
                len(self.waypoint_buffer))

        return rospilot.srv.SetWaypointsResponse()

    def handle_set_rc(self, message):
        if self.allow_control and self.enable_control and \
           self.enable_control_has_been_false:
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
        # See MAV_MODE_FLAG in pymavlink.mavlinkv10
        # self.conn.mav.set_mode_send(self.conn.target_system,
        #        209 if data.armed else 81, 0)

        # So instead we fake the tranmitter signals
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,
            self.conn.target_component, 0, 0,
            1000,  # throttle to zero
            # yaw full right to arm, left to disarm
            2000 if data.armed else 1000,
            0, 0, 0, 0)
        rospy.sleep(5)
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,
            self.conn.target_component, 0, 0, 0, 0, 0, 0, 0, 0)

        return rospilot.srv.SetBasicModeResponse()

    def request_waypoints(self):
        if self.waypoint_read_in_progress or self.waypoint_write_in_progress:
            return
        self.conn.mav.mission_request_list_send(
            self.conn.target_system,
            self.conn.target_component)
        self.waypoint_read_in_progress = True

    def run(self):
        rospy.loginfo("Waiting for heartbeat")
        self.conn.wait_heartbeat()
        rospy.loginfo("Got heartbeat. Waiting 10secs for APM to be ready")
        rospy.sleep(10)

        self.conn.mav.request_data_stream_send(
            self.conn.target_system,
            self.conn.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL,
            self.rate, 1)
        # Send request to read waypoints
        self.request_waypoints()
        last_waypoint_read = time()
        while not rospy.is_shutdown():
            rospy.sleep(0.001)
            msg = self.conn.recv_match(blocking=True)
            if time() - last_waypoint_read > 10:
                last_waypoint_read = time()
                self.request_waypoints()
            if not msg:
                continue
            msg_type = msg.get_type()
            if msg_type == "BAD_DATA":
                rospy.logwarn("Got bad data")
                continue
            if self.export_conn:
                self.export_conn.mav.send(msg)

            if msg_type == "ATTITUDE":
                self.pub_attitude.publish(
                    msg.roll, msg.pitch, msg.yaw,
                    msg.rollspeed, msg.pitchspeed, msg.yawspeed)
            elif msg_type == "RC_CHANNELS_RAW":
                self.pub_rcstate.publish([
                    msg.chan1_raw, msg.chan2_raw,
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
                    msg.lat / float(10 * 1000 * 1000),
                    msg.lon / float(10 * 1000 * 1000),
                    msg.alt / float(1000), msg.satellites_visible)
            elif msg_type == "RAW_IMU":
                self.pub_imuraw.publish(
                    msg.time_usec,
                    Vector3(msg.xgyro / 100.0, msg.ygyro / 100.0, msg.zgyro / 100.0),
                    Vector3(msg.xacc / 100.0, msg.yacc / 100.0, msg.zacc / 100.0),
                    Vector3(msg.xmag / 100.0, msg.ymag / 100.0, msg.zmag / 100.0))
            elif msg_type == "MISSION_COUNT":
                if not self.waypoint_read_in_progress:
                    rospy.logwarn("Did not expect MISSION_COUNT message")
                else:
                    self.num_waypoints = msg.count
                    self.waypoint_buffer = []
                    # Ignore the first one, because it's some magic waypoint
                    if msg.count > 1:
                        # Request the first waypoint
                        self.conn.mav.mission_request_send(
                            self.conn.target_system,
                            self.conn.target_component,
                            1)
            elif msg_type == "MISSION_REQUEST":
                if not self.waypoint_write_in_progress:
                    rospy.logwarn("Waypoint write not in progress, but received a request for a waypoint")
                else:
                    waypoint = self.waypoint_buffer[msg.seq]
                    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                    if msg.seq == 0:
                        # Waypoint zero seems to be special, and uses the
                        # GLOBAL frame. It also is magically reset in the
                        # firmware, so this probably doesn't matter.
                        frame = mavutil.mavlink.MAV_FRAME_GLOBAL
                    self.conn.mav.mission_item_send(
                        self.conn.target_system,
                        self.conn.target_component,
                        msg.seq,
                        frame,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        1 if msg.seq == 0 else 0,  # Set current
                        1,  # Auto continue after this waypoint
                        1.0,  # "reached waypoint" is +/- 1.0m
                        5.0,  # Stay for 5 secs then move on
                        1.0,  # Stay within 1.0m for LOITER
                        0,  # Face north on arrival
                        waypoint.latitude,  # Latitude
                        waypoint.longitude,  # Longitude
                        waypoint.altitude)  # Altitude
            elif msg_type == "MISSION_ACK":
                if not self.waypoint_write_in_progress:
                    rospy.logwarn("Did not expect MISSION_ACK no write in progress")
                # NOTE: APM is suppose to return MAV_CMD_ACK_OK, but it seems
                # to return 0
                elif msg.type not in (0, mavutil.mavlink.MAV_CMD_ACK_OK):
                    rospy.logerr("Bad MISSION_ACK: %d", msg.type)
                    self.waypoint_write_in_progress = False
                else:
                    # All waypoints have been sent, read them back
                    self.waypoint_write_in_progress = False
                    self.conn.mav.mission_request_list_send(
                        self.conn.target_system,
                        self.conn.target_component)
                    self.waypoint_read_in_progress = True
            elif msg_type == "MISSION_ITEM":
                if not self.waypoint_read_in_progress:
                    rospy.logwarn("Did not expect MISSION_ITEM, no read in progress")
                else:
                    self.waypoint_buffer.append(rospilot.msg.Waypoint(msg.x, msg.y, msg.z))
                    if self.num_waypoints == msg.seq + 1:
                        self.conn.mav.mission_ack_send(
                            self.conn.target_system,
                            self.conn.target_component,
                            mavutil.mavlink.MAV_CMD_ACK_OK)
                        self.pub_waypoints.publish(self.waypoint_buffer)
                        self.waypoint_read_in_progress = False
                    else:
                        self.conn.mav.mission_request_send(
                            self.conn.target_system,
                            self.conn.target_component,
                            msg.seq + 1)

if __name__ == '__main__':
    parser = OptionParser("rospilot.py <options>")
    parser.add_option(
        "--baudrate", dest="baudrate",
        type='int', help="serial port baud rate", default=115200)
    parser.add_option(
        "--allow-control", dest="allow_control",
        help="allow sending control signals to autopilot", default="false")
    parser.add_option(
        "--device", dest="device",
        default=None, help="serial device")
    parser.add_option(
        "--udp-export", dest="export_host",
        default=None, help="UDP host/port to send copy of MAVLink data to")
    (opts, args) = parser.parse_args()

    rospy.init_node('rospilot_mavlink')
    node = None
    while not rospy.is_shutdown() and node is None:
        try:
            node = MavlinkNode(
                device=opts.device, baudrate=opts.baudrate,
                export_host=opts.export_host, allow_control=opts.allow_control)
        except SerialException as e:
            rospy.logerr("Failed to initialize mavlink node: " + str(e))
            rospy.sleep(5)
    if node:
        node.run()
