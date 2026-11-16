#!/usr/bin/env python3
"""The operator station's display: the robot's cameras and what the leader arms and the recorder are doing.

Made for recording demonstrations away from the Mac (docs/operator-station.md). It draws the head camera large,
for pointing the head, and the two wrist cameras beside it. The bar underneath shows:
  - whether the leader arms are engaged (/leader_arms/engaged, from leader_arms), and when leader_arms' guard
    stopped them, why (/leader_arms/status: a stuck or overheating servo, flashed over the head image too), and
    any servo running hot;
  - the recorder, record.py on the Mac: episode, seconds recorded, paused and why, the reset between episodes,
    stopped (/recorder/status);
  - which gamepad buttons drive the recorder.
A camera whose frames stop coming gets a red frame and the age of its last frame.

Over the head image: the head's pan and tilt in degrees (/joint_states), and a scale for each between the joint's
limits (/robot_description). The pan scale runs along the bottom with its left end turned fully left, the tilt scale
along the right edge with up at the top, so the marker moves the way the view does.

The recorder is driven from here, by keys in this window or gamepad buttons: both publish /recorder/command (keep,
redo, stop), which record.py treats like the keys in its own terminal.
  keep   end the episode now and keep it; during the reset, end the reset      right arrow, enter / tap keep button
  redo   throw the episode away and record it again                             left arrow, r / redo button
  stop   keep the episode and stop recording                                    q / hold the keep button 1.5 s
f toggles full screen. Closing the window or ctrl-c quits.

The cameras are the light copies cameras.launch.py makes for viewers (<camera>/viewer/compressed, JPEG at
viewer_fps 10), so three of them cost about 1.3 MB/s of wifi on top of what the recorder pulls. For smoother
head pointing, head_topic can be the full-rate stream (/cameras/head/color/image_raw/compressed, 30 fps).

pygame draws the window (SDL: Wayland or X11 on rosie, Cocoa on the Mac). rclpy spins in a background thread and
the callbacks only keep the newest message; the window decodes and draws in the main thread, which macOS requires.
"""
import io
import json
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState, Joy
from std_msgs.msg import Bool, String

# Recorder buttons per joy_layout: (keep, redo) button numbers and the names printed in the bar, plus the name of the
# leader arms' engage button (its number is station.launch.py's ENGAGE_BUTTON, which leader_arms gets).
#   sdl    the Mac's joy_node: back=4 start=6 (joystick_sdl.yaml, RB=10 checked 2026-11-13)
#   linux  evdev order, joystick.yaml's pad: select=10 start=11 (from the evdev key order, not checked)
#   ps     the 8BitDo in PlayStation mode on rosie's evdev: select=8 start=9 (share and options on a PlayStation pad).
#          Named with the 8BitDo's own labels, Nintendo-style letters: X is the top face button (triangle, 2), checked
#          2026-11-16 (docs/operator-station.md).
RECORDER_BUTTONS = {
    "sdl": ((6, 4), ("start", "back"), "Y"),
    "linux": ((11, 10), ("start", "select"), "Y"),
    "ps": ((9, 8), ("start", "select"), "X (top)"),
}
HOLD_TO_STOP_S = 1.5
# The head's joints: positive pan turns it left, positive tilt looks up (head_teleop, checked on the cart 2026-11-14).
HEAD_JOINTS = ("head_pan_joint", "head_tilt_joint")

BG = (14, 14, 16)
TILE = (32, 32, 36)
TEXT = (232, 232, 232)
DIM = (150, 150, 156)
GREEN = (36, 150, 64)
RED = (200, 40, 40)
AMBER = (214, 140, 24)
BLUE = (46, 104, 196)
GREY = (78, 78, 84)


class Camera:
    def __init__(self, name, topic):
        self.name, self.topic = name, topic
        self.data = None          # newest JPEG
        self.arrival = None       # time.monotonic() of the newest frame
        self.seq = 0
        self.drawn_seq = -1
        self.surface = None       # the newest decoded frame
        self.scaled = None        # (size, surface) cache
        self.arrivals = deque(maxlen=90)

    def fps(self, now):
        """Frames in the last 2 s, per second. Call it holding the viewer's lock."""
        recent = [t for t in self.arrivals if now - t < 2.0]
        return len(recent) / 2.0


class StationViewer(Node):
    def __init__(self):
        super().__init__("station_viewer")
        p = self.declare_parameter
        p("head_topic", "/cameras/head/viewer/compressed")
        p("left_topic", "/cameras/left/viewer/compressed")
        p("right_topic", "/cameras/right/viewer/compressed")
        p("fullscreen", False)
        p("window_size", [1280, 640])
        p("joy_layout", "linux")
        p("keep_button", -1)          # -1: from joy_layout
        p("redo_button", -1)
        p("stale_s", 1.0)            # a camera with no frame for this long is drawn as stale
        p("snapshot_path", "")        # a PNG of the window every 2 s, to check it from elsewhere ("" = off)
        gp = lambda name: self.get_parameter(name).value  # noqa: E731

        layout = gp("joy_layout")
        if layout not in RECORDER_BUTTONS:
            raise ValueError(f"joy_layout must be one of {sorted(RECORDER_BUTTONS)}, not {layout!r}")
        (keep, redo), (self.keep_name, self.redo_name), self.engage_name = RECORDER_BUTTONS[layout]
        self.keep_button = gp("keep_button") if gp("keep_button") >= 0 else keep
        self.redo_button = gp("redo_button") if gp("redo_button") >= 0 else redo
        self.fullscreen = gp("fullscreen")
        self.window_size = tuple(int(v) for v in gp("window_size"))
        self.stale_s = gp("stale_s")
        self.snapshot_path = gp("snapshot_path")

        self.lock = threading.Lock()
        self.cameras = [Camera("head", gp("head_topic")), Camera("left wrist", gp("left_topic")),
                        Camera("right wrist", gp("right_topic"))]
        self.engaged = None           # None until leader_arms says
        self.arms_status = None       # leader_arms' guard: {"fault": ..., "fault_seq": ..., "hot": {...}}
        self.alert = None             # (text, until): a guard trip, flashed over the head image
        self.status = None            # the recorder's last status (dict)
        self.status_arrival = None
        self.banner = None            # (text, until): the recorder's last spoken prompt
        self.last_seq = None
        self.keep_down_at = None      # time the keep button went down, None while up
        self.stop_sent = False
        self.last_buttons = []
        self.head_pose = None         # (pan, tilt) in rad, from /joint_states
        self.head_pose_arrival = None
        self.head_limits = {}         # joint -> (lower, upper) in rad, from /robot_description

        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for cam in self.cameras:
            self.create_subscription(CompressedImage, cam.topic, lambda m, c=cam: self.on_frame(c, m), image_qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/leader_arms/engaged", self.on_engaged, latched)
        self.create_subscription(String, "/recorder/status", self.on_status, latched)
        self.create_subscription(String, "/leader_arms/status", self.on_arms_status, latched)
        self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.command_pub = self.create_publisher(String, "/recorder/command", 10)
        # The latched engaged state outlives a leader_arms that was killed, so also check it is still there.
        self.leader_alive = False
        self.create_timer(1.0, self.check_graph)
        self.get_logger().info(
            f"station viewer: {', '.join(c.topic for c in self.cameras)}; recorder buttons "
            f"{self.keep_name}={self.keep_button} (hold: stop), {self.redo_name}={self.redo_button}")

    # -- ROS callbacks (executor thread): keep the newest of everything

    def on_frame(self, cam, msg):
        now = time.monotonic()
        with self.lock:
            cam.data = bytes(msg.data)
            cam.arrival = now
            cam.seq += 1
            cam.arrivals.append(now)

    def on_engaged(self, msg):
        with self.lock:
            self.engaged = msg.data

    def on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in HEAD_JOINTS):
            with self.lock:
                self.head_pose = tuple(pos[j] for j in HEAD_JOINTS)
                self.head_pose_arrival = time.monotonic()

    def on_description(self, msg):
        try:
            root = ET.fromstring(msg.data)
        except ET.ParseError as e:
            self.get_logger().warn(f"robot_description does not parse: {e}")
            return
        limits = {}
        for joint in root.iter("joint"):
            limit = joint.find("limit")
            if joint.get("name") in HEAD_JOINTS and limit is not None:
                limits[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
        with self.lock:
            self.head_limits = limits

    def on_arms_status(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            old = self.arms_status
            # a new trip, not the latched one from before the viewer started
            if old is not None and status.get("fault") and status.get("fault_seq") != old.get("fault_seq"):
                self.alert = (f"Arms stopped: {status['fault']}", time.monotonic() + 5.0)
            self.arms_status = status

    def check_graph(self):
        alive = self.count_publishers("/leader_arms/engaged") > 0
        with self.lock:
            self.leader_alive = alive

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.status = status
            self.status_arrival = time.monotonic()
            seq = status.get("seq")
            if seq != self.last_seq and status.get("prompt"):
                if self.last_seq is not None:   # not the latched one from before the viewer started
                    self.banner = (status["prompt"], time.monotonic() + 2.5)
            self.last_seq = seq

    def on_joy(self, msg):
        b = list(msg.buttons)
        pressed = lambda i: i < len(b) and b[i] == 1  # noqa: E731
        was = lambda i: i < len(self.last_buttons) and self.last_buttons[i] == 1  # noqa: E731
        now = time.monotonic()
        if pressed(self.redo_button) and not was(self.redo_button):
            self.command("redo")
        if pressed(self.keep_button):
            if self.keep_down_at is None:
                self.keep_down_at, self.stop_sent = now, False
            elif not self.stop_sent and now - self.keep_down_at >= HOLD_TO_STOP_S:
                self.command("stop")
                self.stop_sent = True
        elif self.keep_down_at is not None:
            if not self.stop_sent:
                self.command("keep")
            self.keep_down_at = None
        self.last_buttons = b

    def command(self, what):
        self.command_pub.publish(String(data=what))
        self.get_logger().info(f"recorder: {what}")

    # -- the window (main thread)

    def run_window(self):
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame

        pygame.init()
        pygame.display.set_caption("XLeRobot station")
        screen = self._set_mode(pygame)
        clock = pygame.time.Clock()
        fonts = {}
        last_snapshot = 0.0
        while rclpy.ok():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                if event.type == pygame.VIDEORESIZE and not self.fullscreen:
                    self.window_size = event.size
                    screen = self._set_mode(pygame)
                if event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_RIGHT, pygame.K_RETURN, pygame.K_KP_ENTER, pygame.K_n):
                        self.command("keep")
                    elif event.key in (pygame.K_LEFT, pygame.K_r):
                        self.command("redo")
                    elif event.key == pygame.K_q:
                        self.command("stop")
                    elif event.key == pygame.K_f:
                        self.fullscreen = not self.fullscreen
                        screen = self._set_mode(pygame)
            self._draw(pygame, screen, fonts)
            pygame.display.flip()
            if self.snapshot_path and time.monotonic() - last_snapshot > 2.0:
                last_snapshot = time.monotonic()
                pygame.image.save(screen, self.snapshot_path + ".tmp.png")
                os.replace(self.snapshot_path + ".tmp.png", self.snapshot_path)
            clock.tick(30)

    def _set_mode(self, pygame):
        if self.fullscreen:
            screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
            pygame.mouse.set_visible(False)
        else:
            screen = pygame.display.set_mode(self.window_size, pygame.RESIZABLE)
            pygame.mouse.set_visible(True)
        return screen

    def _font(self, pygame, fonts, size):
        if size not in fonts:
            fonts[size] = pygame.font.Font(None, size)
        return fonts[size]

    def _draw(self, pygame, screen, fonts):
        now = time.monotonic()
        W, H = screen.get_size()
        screen.fill(BG)
        bar_h = max(34, H // 12)
        rects = [pygame.Rect(*map(int, r)) for r in self._tiles(W, H - bar_h)]
        head_rect = rects[0]
        drawn = [self._draw_camera(pygame, screen, fonts, cam, rect, now) for cam, rect in zip(self.cameras, rects)]
        self._draw_head_pose(pygame, screen, fonts, drawn[0] or head_rect, now)
        with self.lock:
            banner = self.banner if self.banner and self.banner[1] > now else None
            alert = self.alert if self.alert and self.alert[1] > now else None
        if alert:   # a guard trip wins over the recorder's prompt
            self._draw_banner(pygame, screen, fonts, alert[0], head_rect, bg=(150, 20, 20, 220))
        elif banner:
            self._draw_banner(pygame, screen, fonts, banner[0], head_rect)
        self._draw_bar(pygame, screen, fonts, pygame.Rect(0, H - bar_h, W, bar_h), now)

    @staticmethod
    def _tiles(W, H):
        """[head, left wrist, right wrist] as (x, y, w, h). Head large, wrists half size: beside it on a wide
        screen, under it on a squarer one (for 4:3 frames)."""
        gap = 6
        # side by side: total 2h wide by h tall
        h1 = min(H, (W - gap) / 2.0)
        # stacked: head w wide, wrists below it side by side; total w wide by w*9/8 tall
        w2 = min(W, (H - gap) * 8.0 / 9.0)
        if h1 * 4 / 3 >= w2:
            hw, hh = h1 * 4 / 3, h1
            x0, y0 = (W - (hw + gap + hw / 2)) / 2, (H - hh) / 2
            head = (x0, y0, hw, hh)
            wrists = [(x0 + hw + gap, y0, hw / 2, (hh - gap) / 2),
                      (x0 + hw + gap, y0 + (hh + gap) / 2, hw / 2, (hh - gap) / 2)]
        else:
            hw, hh = w2, w2 * 3 / 4
            x0, y0 = (W - hw) / 2, (H - (hh + gap + hh / 2)) / 2
            head = (x0, y0, hw, hh)
            wrists = [(x0, y0 + hh + gap, (hw - gap) / 2, hh / 2),
                      (x0 + (hw + gap) / 2, y0 + hh + gap, (hw - gap) / 2, hh / 2)]
        return [head] + wrists

    def _draw_camera(self, pygame, screen, fonts, cam, rect, now):
        """Draws the camera into rect. Returns the image's rect, None while there is no frame yet."""
        pygame.draw.rect(screen, TILE, rect)
        with self.lock:   # on_frame appends to cam.arrivals from the executor thread
            data, seq, arrival, fps = cam.data, cam.seq, cam.arrival, cam.fps(now)
        if data is not None and seq != cam.drawn_seq:
            try:
                cam.surface = pygame.image.load(io.BytesIO(data), "frame.jpg").convert()
                cam.scaled = None
            except pygame.error as e:
                self.get_logger().warn(f"{cam.name}: frame does not decode: {e}", throttle_duration_sec=5.0)
            cam.drawn_seq = seq
        font = self._font(pygame, fonts, max(18, rect.height // 14))
        if cam.surface is None:
            msg = font.render(f"{cam.name}: waiting for {cam.topic}", True, DIM)
            screen.blit(msg, msg.get_rect(center=rect.center))
            return None
        sw, sh = cam.surface.get_size()
        scale = min(rect.width / sw, rect.height / sh)
        size = (max(1, int(sw * scale)), max(1, int(sh * scale)))
        if cam.scaled is None or cam.scaled[0] != size:
            cam.scaled = (size, pygame.transform.smoothscale(cam.surface, size))
        img_rect = cam.scaled[1].get_rect(center=rect.center)
        screen.blit(cam.scaled[1], img_rect)
        age = now - arrival
        label = f"{cam.name}  {fps:.0f} fps"
        colour = TEXT
        if age > self.stale_s:
            pygame.draw.rect(screen, RED, img_rect, width=max(3, rect.height // 80))
            label = f"{cam.name}  no frames for {age:.0f} s"
            colour = RED
        text = font.render(label, True, colour)
        shade = pygame.Surface((text.get_width() + 12, text.get_height() + 6), pygame.SRCALPHA)
        shade.fill((0, 0, 0, 150))
        screen.blit(shade, (img_rect.x + 6, img_rect.y + 6))
        screen.blit(text, (img_rect.x + 12, img_rect.y + 9))
        return img_rect

    @staticmethod
    def _shade(pygame, screen, box, alpha=150):
        shade = pygame.Surface(box.size, pygame.SRCALPHA)
        shade.fill((0, 0, 0, alpha))
        screen.blit(shade, box)

    def _draw_head_pose(self, pygame, screen, fonts, img, now):
        """The head's pan and tilt over its image: degrees in the top right corner, and a scale for each between the
        joint's limits, pan along the bottom (left end = turned fully left) and tilt down the right edge (up at the
        top)."""
        with self.lock:
            pose, arrival, limits = self.head_pose, self.head_pose_arrival, dict(self.head_limits)
        font = self._font(pygame, fonts, max(18, img.height // 16))
        if pose is None or now - arrival > self.stale_s:
            lines = [("pan / tilt: no joint states", DIM)]
            pose = None
        else:
            def words(rad, plus, minus):
                deg = math.degrees(rad)
                side = plus if deg >= 0.5 else minus if deg <= -0.5 else ""
                return f"{abs(deg):.0f}\N{DEGREE SIGN} {side}".rstrip()
            lines = [(f"pan  {words(pose[0], 'left', 'right')}", TEXT), (f"tilt  {words(pose[1], 'up', 'down')}", TEXT)]
        texts = [font.render(t, True, c) for t, c in lines]
        box = pygame.Rect(0, 0, max(t.get_width() for t in texts) + 16, sum(t.get_height() for t in texts) + 10)
        box.topright = (img.right - 6, img.y + 6)
        self._shade(pygame, screen, box)
        y = box.y + 5
        for t in texts:
            screen.blit(t, (box.x + 8, y))
            y += t.get_height()
        if pose is None:
            return
        size = max(10, img.height // 30)
        small = self._font(pygame, fonts, max(14, img.height // 26))
        pan_limits, tilt_limits = (limits.get(j) for j in HEAD_JOINTS)
        if pan_limits:
            y = img.bottom - 2 * size
            self._draw_scale(pygame, screen, small, (img.x + img.width * 0.3, y), (img.right - img.width * 0.3, y),
                             pan_limits, pose[0], ("L", "R"), size)
        if tilt_limits:
            x = img.right - 2 * size
            self._draw_scale(pygame, screen, small, (x, img.y + img.height * 0.28), (x, img.bottom - img.height * 0.2),
                             tilt_limits, pose[1], ("up", "down"), size)

    def _draw_scale(self, pygame, screen, font, start, end, limits, value, names, size):
        """A scale from start (the joint's upper limit) to end (its lower limit), a tick at 0 and a marker at value.
        names label the two ends."""
        lo, hi = limits
        if hi <= lo:
            return
        along = lambda v: (hi - min(max(v, lo), hi)) / (hi - lo)  # noqa: E731
        point = lambda f: (start[0] + (end[0] - start[0]) * f, start[1] + (end[1] - start[1]) * f)  # noqa: E731
        horizontal = start[1] == end[1]
        labels = [font.render(n, True, DIM) for n in names]
        if horizontal:   # the end labels sit beyond the ends
            box = pygame.Rect(start[0] - labels[0].get_width() - size, start[1] - size,
                              end[0] - start[0] + labels[0].get_width() + labels[1].get_width() + 2 * size, 2 * size)
        else:            # above and below
            box = pygame.Rect(start[0] - size, start[1] - labels[0].get_height() - size // 2,
                              2 * size, end[1] - start[1] + labels[0].get_height() + labels[1].get_height() + size)
            box.inflate_ip(max(0, max(lb.get_width() for lb in labels) + 8 - box.width), 0)
        self._shade(pygame, screen, box.inflate(8, 6), alpha=120)
        line_w = max(2, size // 5)
        pygame.draw.line(screen, DIM, start, end, line_w)
        for f, length in ((0.0, size * 0.6), (1.0, size * 0.6), (along(0.0), size * 1.2)):
            x, y = point(f)
            if horizontal:
                pygame.draw.line(screen, TEXT, (x, y - length / 2), (x, y + length / 2), line_w)
            else:
                pygame.draw.line(screen, TEXT, (x - length / 2, y), (x + length / 2, y), line_w)
        x, y = point(along(value))
        pygame.draw.circle(screen, AMBER, (int(x), int(y)), max(4, int(size * 0.45)))
        if horizontal:
            screen.blit(labels[0], labels[0].get_rect(midright=(start[0] - size // 2, start[1])))
            screen.blit(labels[1], labels[1].get_rect(midleft=(end[0] + size // 2, end[1])))
        else:
            screen.blit(labels[0], labels[0].get_rect(midbottom=(start[0], start[1] - size // 3)))
            screen.blit(labels[1], labels[1].get_rect(midtop=(end[0], end[1] + size // 3)))

    def _draw_banner(self, pygame, screen, fonts, text, rect, bg=(0, 0, 0, 190)):
        font = self._font(pygame, fonts, max(28, rect.height // 9))
        msg = font.render(text, True, TEXT)
        if msg.get_width() > rect.width - 60:   # a long guard message: smaller, to fit the head image
            font = self._font(pygame, fonts, max(18, int(font.get_height() * (rect.width - 60) / msg.get_width())))
            msg = font.render(text, True, TEXT)
        box = msg.get_rect(center=rect.center).inflate(40, 24)
        shade = pygame.Surface(box.size, pygame.SRCALPHA)
        shade.fill(bg)
        screen.blit(shade, box)
        screen.blit(msg, msg.get_rect(center=rect.center))

    def _draw_bar(self, pygame, screen, fonts, bar, now):
        font = self._font(pygame, fonts, int(bar.height * 0.6))
        small = self._font(pygame, fonts, int(bar.height * 0.45))
        with self.lock:
            engaged, status, status_arrival = self.engaged, self.status, self.status_arrival
            leader_alive, arms = self.leader_alive, self.arms_status or {}
        pad = bar.height // 5
        x = pad

        # leader arms
        fault = arms.get("fault", "") if leader_alive and not engaged else ""
        if engaged is None or not leader_alive:
            text, bg = "no leader arms", GREY
        elif fault:
            text, bg = "ARMS STOPPED", RED
        else:
            text, bg = ("ARMS ON", GREEN) if engaged else ("ARMS OFF", GREY)
        x = self._pill(pygame, screen, font, text, bg, x, bar, pad) + pad

        # recorder
        text, bg, dot = self._recorder_text(status, status_arrival, now)
        x = self._pill(pygame, screen, font, text, bg, x, bar, pad, dot=dot) + pad

        # why the guard stopped the arms, and hot servos: these push the button hints out when there's no room
        notes = []
        if fault:
            notes.append((fault, (255, 110, 110)))
        hot = arms.get("hot", {}) if leader_alive else {}
        if hot:
            notes.append(("hot: " + ", ".join(f"{n} {t}\N{DEGREE SIGN}C" for n, t in hot.items()), AMBER))
        for note, colour in notes:
            msg = small.render(note, True, colour)
            if x + msg.get_width() + pad > bar.right:
                break
            screen.blit(msg, msg.get_rect(midleft=(x, bar.centery)))
            x += msg.get_width() + 2 * pad

        # button hints, right-aligned, dropped if they don't fit
        hint = (f"{self.keep_name}: keep   hold {self.keep_name}: stop   {self.redo_name}: redo   "
                f"{self.engage_name}: arms on/off")
        msg = small.render(hint, True, DIM)
        if x + msg.get_width() + pad < bar.right:
            screen.blit(msg, msg.get_rect(midright=(bar.right - pad, bar.centery)))

    def _recorder_text(self, status, arrival, now):
        """(text, colour, dot) for the recorder pill."""
        if status is None:
            return "recorder off", GREY, False
        state = status.get("state", "?")
        age = now - arrival
        if state != "stopped" and age > 5.0:
            return f"recorder silent {age:.0f} s", AMBER, False
        ep, t, of = status.get("episode", 0), status.get("t", 0.0), status.get("of", 0.0)
        saved, goal = status.get("saved", 0), status.get("goal", 0)
        note = status.get("note", "")
        if state == "recording":
            return f"REC  ep {ep}  {t:4.1f}/{of:.0f} s   saved {saved}/{goal}", RED, True
        if state == "paused":
            return f"PAUSED  ep {ep}  {t:4.1f}/{of:.0f} s  {note}", AMBER, False
        if state == "reset":
            return f"RESET  {status.get('left', 0.0):.0f} s   saved {saved}/{goal}", BLUE, False
        if state == "saving":
            return f"SAVING  ep {ep}", BLUE, False
        if state == "waiting":
            return f"WAITING  {note}", AMBER, False
        if state == "stopped":
            total = status.get("total", 0)
            return f"STOPPED  {total} episode{'' if total == 1 else 's'} in the dataset", GREY, False
        return f"recorder: {state}", GREY, False

    def _pill(self, pygame, screen, font, text, bg, x, bar, pad, dot=False):
        msg = font.render(text, True, TEXT)
        r = bar.height // 2 - pad // 2
        width = msg.get_width() + 2 * pad + (2 * r if dot else 0)
        width = min(width, bar.right - pad - x)
        box = pygame.Rect(x, bar.y + pad // 2, max(0, width), bar.height - pad)
        pygame.draw.rect(screen, bg, box, border_radius=box.height // 3)
        tx = x + pad
        if dot:
            pygame.draw.circle(screen, TEXT, (tx + r // 2, bar.centery), max(4, r // 2))
            tx += 2 * r
        screen.set_clip(box)
        screen.blit(msg, msg.get_rect(midleft=(tx, bar.centery)))
        screen.set_clip(None)
        return box.right


def main():
    rclpy.init()
    node = StationViewer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    try:
        node.run_window()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
