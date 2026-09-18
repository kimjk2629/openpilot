"""
Shows a small "NDA" badge at the top-center of the onroad view whenever the speed-camera WiFi
bridge (openpilot.sunnypilot.selfdrive.navi.navi_camera_bridge) is enabled and actively connected
to a paired navigation app - i.e. speed camera / section alerts are available right now, whether
or not one happens to be active this moment.
"""
import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

NDA_BADGE_HEIGHT = 48
PARAMS_CHECK_INTERVAL_FRAMES = 20  # toggle rarely changes onroad - no need to check every frame

COLOR_IDLE_FILL = rl.Color(0, 90, 0, 170)      # green: connected, no camera/section active
COLOR_IDLE_BORDER = rl.Color(0, 220, 0, 200)
COLOR_ACTIVE_FILL = rl.Color(0, 60, 150, 190)  # blue: a camera/section alert is active
COLOR_ACTIVE_BORDER = rl.Color(60, 150, 255, 220)


class NdaIndicatorRenderer(Widget):
  def __init__(self):
    super().__init__()
    self.params = Params()
    self.font_bold = gui_app.font(FontWeight.BOLD)
    self.enabled = False
    self.connected = False
    self.active = False
    self._frame = 0

  def update(self) -> None:
    self._frame += 1
    if self._frame % PARAMS_CHECK_INTERVAL_FRAMES == 0:
      self.enabled = self.params.get_bool("EnableCameraSpeedLimit")

    self.connected = self.params.get_bool("CameraSpeedLimitConnected") if self.enabled else False
    self.active = self.params.get_bool("CameraSpeedLimitActive") if self.connected else False

  @property
  def visible(self) -> bool:
    return self.enabled and self.connected

  @property
  def height(self) -> float:
    return NDA_BADGE_HEIGHT if self.visible else 0.

  def _render(self, rect: rl.Rectangle) -> None:
    if not self.visible:
      return

    text = "NDA"
    text_size = measure_text_cached(self.font_bold, text, 34)
    badge_width = text_size.x + 36

    fill_color = COLOR_ACTIVE_FILL if self.active else COLOR_IDLE_FILL
    border_color = COLOR_ACTIVE_BORDER if self.active else COLOR_IDLE_BORDER

    badge_rect = rl.Rectangle(rect.x + rect.width / 2 - badge_width / 2, rect.y - 4, badge_width, NDA_BADGE_HEIGHT)
    rl.draw_rectangle_rounded(badge_rect, 0.3, 10, fill_color)
    rl.draw_rectangle_rounded_lines_ex(badge_rect, 0.3, 10, 2, border_color)

    origin = rl.Vector2(badge_rect.x + badge_rect.width / 2 - text_size.x / 2,
                         badge_rect.y + badge_rect.height / 2 - text_size.y / 2)
    rl.draw_text_ex(self.font_bold, text, origin, 34, 0, rl.Color(255, 255, 255, 230))
