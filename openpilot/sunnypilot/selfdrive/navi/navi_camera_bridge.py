#!/usr/bin/env python3
"""
Speed camera / road speed-limit bridge.

Receives speed-camera and road speed-limit alerts broadcast over the local WiFi network by a
paired navigation phone app, using the same UDP discovery/JSON protocol as the well-known
OPKR/neokii "road limit service" (many Korean navigation apps that support that protocol will
work with this unmodified). The currently active camera limit is republished into Params so
sunnypilot's native Speed Limit Assist (openpilot.sunnypilot.selfdrive.controls.lib.speed_limit)
can pick it up as an additional, higher-priority source - no separate deceleration logic needed,
the existing, already-tested Speed Limit Assist state machine handles the actual slow-down.

Security note: the original protocol also supported a 'cmd' / 'echo_cmd' message that let the
paired phone execute arbitrary shell commands on the device with no authentication. That has
been intentionally left out here - only speed-limit / road-limit data is accepted from the
network. Do not add it back without a real auth mechanism.
"""
import fcntl
import json
import select
import socket
import struct
import threading
import time

import numpy as np

from openpilot.cereal import messaging
from openpilot.common.constants import CV
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper

UPDATE_RATE_HZ = 3.0
CAMERA_SPEED_FACTOR_DEFAULT = 1.05
STALE_AFTER_S = 6.0


class Port:
  BROADCAST_PORT = 2899
  RECEIVE_PORT = 3843
  LOCATION_PORT = BROADCAST_PORT


class NaviCameraBridge:
  """Talks UDP with the paired phone app and republishes the active camera limit into Params."""

  def __init__(self):
    self.params = Params()
    self.gps_service = get_gps_location_service(self.params)
    self.sm = messaging.SubMaster([self.gps_service, 'carState'])

    self.terminate = threading.Event()

    self.lock = threading.Lock()
    self.json_road_limit: dict | None = None
    self.last_updated = 0.
    self.last_updated_active = 0.
    self.active = 0
    self.remote_addr = None
    self.remote_gps_addr = None

    self.location = None
    self.gps_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # SpeedLimiter-style ramping state (ported from the neokii road-limit client; this part of
    # the math is plain physics, not car-specific, so it carries over unchanged)
    self.slowing_down = False
    self.started_dist = 0.
    self.last_limit_speed_left_dist = 0.

  # ---- network plumbing (ported from the neokii NaviServer) ----------------------------------

  def get_broadcast_address(self):
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        ip = fcntl.ioctl(s.fileno(), 0x8919, struct.pack('256s', 'wlan0'.encode('utf-8')))[20:24]
        return socket.inet_ntoa(ip)
    except OSError:
      return None

  def broadcast_thread(self):
    broadcast_address = None
    frame = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
      while not self.terminate.is_set():
        try:
          if broadcast_address is None or frame % 10 == 0:
            broadcast_address = self.get_broadcast_address()

          if broadcast_address is not None and self.remote_addr is None:
            msg = 'EON:ROAD_LIMIT_SERVICE:v1'.encode()
            ip_tuple = socket.inet_aton(broadcast_address)
            for i in range(1, 255):
              new_ip = ip_tuple[:-1] + bytes([i])
              sock.sendto(msg, (socket.inet_ntoa(new_ip), Port.BROADCAST_PORT))
        except OSError:
          pass

        time.sleep(5.)
        frame += 1

  def gps_thread(self):
    rk = Ratekeeper(3.0, print_delay_threshold=None)
    while not self.terminate.is_set():
      self._gps_timer()
      rk.keep_time()

  def _gps_timer(self):
    try:
      if self.remote_gps_addr is None:
        return

      if self.sm.updated[self.gps_service]:
        self.location = self.sm[self.gps_service]

      if self.location is not None:
        json_location = json.dumps({"location": [
          self.location.latitude,
          self.location.longitude,
          self.location.altitude,
          self.location.speed,
          self.location.bearingDeg,
          self.location.horizontalAccuracy,
          self.location.unixTimestampMillis,
          self.location.verticalAccuracy,
          self.location.bearingAccuracyDeg,
          self.location.speedAccuracy,
        ]})
        self.gps_socket.sendto(json_location.encode(), (self.remote_gps_addr[0], Port.LOCATION_PORT))
    except (OSError, KeyError):
      self.remote_gps_addr = None

  def sm_update_thread(self):
    rk = Ratekeeper(10, print_delay_threshold=None)
    while not self.terminate.is_set():
      self.sm.update(0)
      rk.keep_time()

  def udp_recv(self, sock) -> None:
    try:
      ready = select.select([sock], [], [], 1.)
      if not ready[0]:
        return

      data, addr = sock.recvfrom(2048)
      self.remote_addr = addr
      json_obj = json.loads(data.decode())

      if 'request_gps' in json_obj:
        self.remote_gps_addr = addr if json_obj['request_gps'] == 1 else None

      with self.lock:
        if 'active' in json_obj:
          self.active = json_obj['active']
          self.last_updated_active = time.monotonic()

        if 'road_limit' in json_obj:
          self.json_road_limit = json_obj['road_limit']
          self.last_updated = time.monotonic()

    except (OSError, ValueError, UnicodeDecodeError):
      with self.lock:
        self.json_road_limit = None

  def send_sdp(self, sock) -> None:
    if self.remote_addr is not None:
      try:
        sock.sendto('EON:ROAD_LIMIT_SERVICE:v1'.encode(), (self.remote_addr[0], Port.BROADCAST_PORT))
      except OSError:
        pass

  def check_stale(self) -> None:
    now = time.monotonic()
    if now - self.last_updated > STALE_AFTER_S:
      with self.lock:
        self.json_road_limit = None
    if now - self.last_updated_active > STALE_AFTER_S:
      self.active = 0
      self.remote_addr = None

  def get_limit_val(self, key, default=None):
    with self.lock:
      j = self.json_road_limit
    if j is None:
      return default
    return j.get(key, default)

  # ---- camera/section limit -> effective target speed (ported from SpeedLimiter.get_max_speed) --

  def compute_camera_limit(self, v_ego: float) -> tuple[bool, float, float]:
    """Returns (active, limit_speed_ms, left_dist_m) for the currently relevant camera/section."""

    if not self.active:
      self.slowing_down = False
      return False, 0., 0.

    is_highway = self.get_limit_val("is_highway")
    cam_type = int(self.get_limit_val("cam_type", 0) or 0)
    cam_limit_speed_left_dist = self.get_limit_val("cam_limit_speed_left_dist", 0) or 0
    cam_limit_speed = self.get_limit_val("cam_limit_speed", 0) or 0
    section_limit_speed = self.get_limit_val("section_limit_speed", 0) or 0
    section_left_dist = self.get_limit_val("section_left_dist", 0) or 0
    section_avg_speed = self.get_limit_val("section_avg_speed", 0) or 0
    section_adjust_speed = self.get_limit_val("section_adjust_speed", False)
    cam_speed_factor = float(np.clip(self.get_limit_val("cam_speed_factor", CAMERA_SPEED_FACTOR_DEFAULT), 1.0, 1.1))

    if is_highway:
      min_limit, max_limit = 40, 120
    else:
      min_limit, max_limit = 20, 100
    if cam_type == 22:  # speed bump
      min_limit = 10

    v_ego_kph = v_ego * CV.MS_TO_KPH

    if cam_limit_speed_left_dist > 0 and cam_limit_speed > 0:
      diff_speed_kph = v_ego_kph - cam_limit_speed * cam_speed_factor

      if cam_type == 22:
        safe_dist, starting_dist = v_ego * 4., v_ego * 8.
      else:
        safe_dist, starting_dist = v_ego * 7., v_ego * 30.

      if self.slowing_down and self.last_limit_speed_left_dist - cam_limit_speed_left_dist < -(v_ego * 5):
        self.slowing_down = False

      if min_limit <= cam_limit_speed <= max_limit and (self.slowing_down or cam_limit_speed_left_dist < starting_dist):
        if not self.slowing_down:
          self.started_dist = cam_limit_speed_left_dist
          self.slowing_down = True

        td = self.started_dist - safe_dist
        d = cam_limit_speed_left_dist - safe_dist

        pp = (d / td) ** 0.6 if (d > 0. and td > 0. and diff_speed_kph > 0.) else 0.
        self.last_limit_speed_left_dist = cam_limit_speed_left_dist

        limit_kph = cam_limit_speed * cam_speed_factor + pp * diff_speed_kph
        return True, limit_kph * CV.KPH_TO_MS, float(cam_limit_speed_left_dist)

      self.slowing_down = False
      return False, 0., 0.

    if section_left_dist > 0 and section_limit_speed > 0 and min_limit <= section_limit_speed <= max_limit:
      self.slowing_down = True
      speed_diff_kph = 0.
      if section_adjust_speed:
        speed_diff_kph = (section_limit_speed - section_avg_speed) / 2.
        speed_diff_kph *= float(np.interp(section_left_dist, [500, 1000], [0., 1.]))

      limit_kph = section_limit_speed * cam_speed_factor + speed_diff_kph
      return True, limit_kph * CV.KPH_TO_MS, float(section_left_dist)

    self.slowing_down = False
    return False, 0., 0.

  # ---- main loop -------------------------------------------------------------------------------

  def publish_loop(self) -> None:
    rk = Ratekeeper(UPDATE_RATE_HZ, print_delay_threshold=None)
    last_frame_t = time.monotonic()

    while not self.terminate.is_set():
      now = time.monotonic()
      dt = now - last_frame_t
      last_frame_t = now

      v_ego = self.sm['carState'].vEgo if self.sm.valid['carState'] else 0.

      active, limit_ms, left_dist = self.compute_camera_limit(v_ego)

      # Decay the remaining distance between phone-app updates so the target ramps smoothly
      # instead of jumping every ~0.33s when a fresh UDP packet lands.
      if active and left_dist > 0:
        left_dist = max(left_dist - v_ego * dt, 0.)

      # "Connected" means the paired phone app is live and sending road-limit updates at all -
      # used for a persistent "NDA" style status badge, independent of whether a camera/section
      # is actually active right now.
      self.params.put_bool("CameraSpeedLimitConnected", bool(self.active))

      self.params.put_bool("CameraSpeedLimitActive", active)
      if active:
        self.params.put("CameraSpeedLimit", round(limit_ms, 2))
        self.params.put("CameraSpeedLimitDistance", round(left_dist, 1))

      self.check_stale()
      rk.keep_time()

  def run(self) -> None:
    threading.Thread(target=self.broadcast_thread, daemon=True).start()
    threading.Thread(target=self.sm_update_thread, daemon=True).start()
    threading.Thread(target=self.gps_thread, daemon=True).start()

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(('0.0.0.0', Port.RECEIVE_PORT))

    def recv_loop():
      while not self.terminate.is_set():
        self.udp_recv(recv_sock)
        self.send_sdp(recv_sock)

    threading.Thread(target=recv_loop, daemon=True).start()

    try:
      self.publish_loop()
    finally:
      self.terminate.set()
      recv_sock.close()


def main():
  # Reset stale state from a previous drive so a crash/restart never leaves a phantom camera
  # limit active until the next UDP packet arrives.
  params = Params()
  params.put_bool("CameraSpeedLimitConnected", False)
  params.put_bool("CameraSpeedLimitActive", False)
  params.put("CameraSpeedLimit", 0.0)
  params.put("CameraSpeedLimitDistance", 0.0)

  NaviCameraBridge().run()


if __name__ == "__main__":
  main()
