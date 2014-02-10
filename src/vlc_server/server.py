import vlc
import os
import sys

class RedirectStdStreams(object):
  '''
  @example:
  vlcs = VlcServer(256, "0.0.0.0", 8080, "/dev/video0", "yuyv", "30", 1280, 720)
  vlcs.play()
  '''
  def __init__(self, stdout=None, stderr=None):
    self._stdout = stdout or sys.stdout
    self._stderr = stderr or sys.stderr

  def __enter__(self):
    self.old_stdout, self.old_stderr = sys.stdout, sys.stderr
    self.old_stdout.flush(); self.old_stderr.flush()
    sys.stdout, sys.stderr = self._stdout, self._stderr

  def __exit__(self, exc_type, exc_value, traceback):
    self._stdout.flush(); self._stderr.flush()
    sys.stdout = self.old_stdout
    sys.stderr = self.old_stderr

# NOTE: This first version is hard coded for WMV2.
class VlcServer:
  def __init__(self, vbit_rate, host, port, device, chroma, fps, width, height):
    self.sout = "#transcode{vcodec=WMV2,vb=%s}:http{dst=%s:%s,mux=wmv}" % (vbit_rate, host, port)
    self.mrl = "v4l2://%s:chroma=%s:fps=%s:width=%s:height=%s" % (device,
                                                                  chroma,
                                                                  fps,
                                                                  width,
                                                                  height)
    self.media_id     = "ros"
    self.vlc_instance = vlc.Instance()
    self.vlc_instance.vlm_add_broadcast(
      "ros", self.mrl, self.sout, 0, None, True, False)

  def play(self):
      self.vlc_instance.vlm_play_media(self.media_id)
  def pause(self):
    self.vlc_instance.vlm_pause_media(self.media_id)
  def stop(self):
    self.vlc_instance.vlm_stop_media(self.media_id)

class VlcRecorder:
  '''
  @example:
  recorder = VlcRecorder("http://0.0.0.0:8080", "/tmp/bordicon.wmv")
  '''
  def __init__(self, mrl, mux):
    self.mux          = mux
    self.vlc_instance = vlc.Instance()
    self.vlc_media    = vlc.libvlc_media_new_location(self.vlc_instance, mrl)
    self.vlc_player   = vlc.libvlc_media_player_new_from_media(self.vlc_media)

  @classmethod
  def generate_sout(self, mux, dest):
    return ':sout=#standard{mux=%s,dst=%s,access=file}' % (mux, dest)

  def start_record(self, dest):
    vlc.libvlc_media_add_option(self.vlc_media,
                                VlcRecorder.generate_sout(self.mux, dest))
    vlc.libvlc_media_player_play(self.vlc_player)

  def stop_record(self):
    vlc.libvlc_media_player_stop(self.vlc_player)

  def take_snapshot(self, path, width, height):
    vlc.libvlc_media_add_option(self.vlc_media, ':snapshot-path=%s' % path)
    vlc.libvlc_media_add_option(self.vlc_media, ':snapshot-format=png')
    vlc.libvlc_video_take_snapshot(self.vlc_player, mrl, width, height)
