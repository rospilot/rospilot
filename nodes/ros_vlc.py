import roslib; roslib.load_manifest('rospilot')
import rospy
import rospilot.msg
import vlc_server

class VlcNode:
  def __init__(self):
    self.server = vlc_server.VlcServer(vbit_rate  = 256,
                                       host       = "0.0.0.0",
                                       port       = 8080
                                       device     = "/dev/video0",
                                       chroma     = "yuyv",
                                       fps        = 30,
                                       width      = 1280,
                                       height     = 720)
    self.recorder = vlc_server.VlcRecorder(mrl        = "http://0.0.0.0:8080",
                                           dest_path  = "/tmp/ros_vid.mwv")
    rospy.init_node("ros_vlc")
    rospy.Service('play_stream',  std_srvs.srv.Empty, self.handle_play_stream)
    rospy.Service('pause_stream', std_srvs.srv.Empty, self.handle_pause_stream)
    rospy.Service('stop_stream',  std_srvs.srv.Empty, self.handle_stop_stream)

    rospy.Service('start_record', std_srvs.srv.Empty, self.handle_start_record)
    rospy.Service('stop_record',  std_srvs.srv.Empty, self.handle_stop_record)

    def handle_play_stream(self):
        self.server.play()

    def handle_pause_stream(self):
        self.server.pause()

    def handle_stop_stream(self):
        self.server.stop()

    def handle_start_record(self):
        self.recorder.start_record()

    def handle_stop_record(self):
        self.recorder.stop_record()

    def run(self):
        rospy.loginfo("Vlc Node Initialized and waiting...")
        rospy.spin()

if __name__ == '__main__':
    # parser = OptionParser("rospilot.py <options>")
    # parser.add_option(
    #     "--baudrate", dest="baudrate",
    #     type='int', help="serial port baud rate", default=115200)
    node = VlcNode()
    node.run()
