#!/usr/bin/env python
import roslib; roslib.load_manifest('rospilot')
import rospy
from pymavlink import mavutil
import rospilot.msg

from optparse import OptionParser
class MavlinkNode:
    def __init__(self, device, baudrate, export_host):
        self.export_conn = None
        self.rate = 10
        if export_host:
            self.export_conn = mavutil.mavlink_connection(
                    "udp:" + export_host, input=False)
        self.conn = mavutil.mavlink_connection(device, baud=baudrate)
        self.pub_attitude = rospy.Publisher('attitude', rospilot.msg.Attitude)
        self.pub_rcstate = rospy.Publisher('rcstate', rospilot.msg.RCState)

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

if __name__ == '__main__':
    parser = OptionParser("rospilot.py <options>")
    parser.add_option("--baudrate", dest="baudrate", 
            type='int', help="serial port baud rate", default=115200)
    parser.add_option("--device", dest="device", 
            default=None, help="serial device")
    parser.add_option("--udp-export", dest="export_host", 
            default=None, help="UDP host/port to send copy of MAVLink data to")
    (opts, args) = parser.parse_args()

    node = MavlinkNode(device=opts.device, baudrate=opts.baudrate, 
            export_host=opts.export_host)
    node.run()
