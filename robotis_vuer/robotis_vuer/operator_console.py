"""One focused X11 terminal for hand pause, bounded manual motion and Loop keys."""

import argparse
import fcntl
import glob
import json
import math
import os
import select
import selectors
import struct
import sys
import termios
import time
import tty
import uuid

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotis_vuer.loop_keys import X11Keys
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


KEYS = {17: 'w', 31: 's', 30: 'a', 32: 'd', 16: 'q', 18: 'e', 24: 'o', 25: 'p',
        22: 'u', 48: 'b', 46: 'c', 103: 'up', 108: 'down', 105: 'left', 106: 'right'}
SHIFT = {42, 54}
CTRL = {29, 97}
MODIFIERS = CTRL | {56, 100, 125, 126}
EVENT = struct.Struct('@llHHi')
LIMITS = {'head_joint1': (-.2317, .6951), 'head_joint2': (-.35, .35),
          'lift_joint': (-.5, 0.)}


def key_route(key, paused, shift=False, loop_input=False):
    if key == 'u':
        return 'toggle'
    if key in ('a', 'b', 'c') and (loop_input or key != 'a' or not paused or shift):
        return 'loop'
    if paused and key in ('w', 'a', 's', 'd', 'q', 'e', 'o', 'p',
                          'up', 'down', 'left', 'right'):
        return 'motion'
    return 'ignore'


def motion_vector(keys, linear=.15, angular=.3):
    x = linear * (('up' in keys) - ('down' in keys))
    y = linear * (('left' in keys) - ('right' in keys))
    length = math.hypot(x, y)
    if length > linear:
        x, y = x * linear / length, y * linear / length
    return x, y, angular * (('q' in keys) - ('e' in keys))


def bounded_step(measured, target, direction, speed, dt, limits, lead):
    if not all(math.isfinite(v) for v in (measured, target, speed, dt)):
        raise ValueError('Non-finite joint command')
    target = min(measured + lead, max(measured - lead, target + direction * speed * dt))
    return min(limits[1], max(limits[0], target))


class OperatorConsole(Node):
    def __init__(self):
        super().__init__('quest_operator_console')
        self.status = None
        self.status_time = 0.
        self.request_id = None
        self.request_time = 0.
        self.pending_command = None
        self.joints = {}
        self.keys = set()
        self.targets = {}
        self.moving = set()
        self.base_moving = False
        self.last_tick = time.monotonic()
        self.request_pub = self.create_publisher(String, '/vr/operator/request', 10)
        self.status_sub = self.create_subscription(
            String, '/vr/operator/status', self.receive_status, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.receive_joints, qos_profile_sensor_data)
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.head_pub = self.create_publisher(
            JointTrajectory, '/leader/joystick_controller_left/joint_trajectory', 10)
        self.lift_pub = self.create_publisher(
            JointTrajectory, '/leader/joystick_controller_right/joint_trajectory', 10)
        self._last_display = None

    def receive_status(self, msg):
        try:
            status = json.loads(msg.data)
            if not isinstance(status.get('paused'), bool):
                return
        except (ValueError, AttributeError):
            return
        self.status, self.status_time = status, time.monotonic()
        if self.request_id == status.get('request_id'):
            self.pending_command = None
        label = (status.get('paused'), status.get('resuming'), status.get('reason'))
        if label != self._last_display:
            self._last_display = label
            print(f'VR: {status.get("reason")} | U=toggle; paused={status["paused"]}', flush=True)

    def receive_joints(self, msg):
        now = time.monotonic()
        for name, value in zip(msg.name, msg.position):
            if name in LIMITS and math.isfinite(value):
                self.joints[name] = (value, now)

    def request(self, command):
        self.keys.clear()
        self.request_id, self.pending_command = uuid.uuid4().hex, command
        self.request_time = time.monotonic()
        self._send_request()

    def _send_request(self):
        self.request_pub.publish(String(data=json.dumps({
            'id': self.request_id, 'command': self.pending_command})))

    def fresh_status(self):
        return self.status is not None and time.monotonic() - self.status_time < .5

    def toggle(self):
        if not self.fresh_status() or self.pending_command:
            print('U unavailable: waiting for hand publisher status/acknowledgement.', flush=True)
            return
        self.request('resume' if self.status['paused'] else 'pause')

    def manual_allowed(self):
        return (self.fresh_status() and self.status['paused'] and
                not self.pending_command and time.monotonic() - self.request_time >= .2)

    def measured(self, name):
        value, stamp = self.joints.get(name, (None, 0.))
        return value if time.monotonic() - stamp < .5 else None

    @staticmethod
    def trajectory(names, values):
        msg = JointTrajectory()
        msg.joint_names = names
        point = JointTrajectoryPoint(positions=[float(v) for v in values])
        point.time_from_start.nanosec = 120000000
        msg.points = [point]
        return msg

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(.05, now - self.last_tick), now
        if self.pending_command:
            if now - self.request_time < 2.:
                self._send_request()  # Idempotent request ID; tolerate discovery delay.
            else:
                print('Pause/resume acknowledgement timed out; manual motion disabled.',
                      flush=True)
                self.pending_command = None
                self.status = None
        allowed = self.manual_allowed()
        if not allowed:
            self.keys.clear()
        keys = self.keys if allowed else set()
        velocity = motion_vector(keys)
        moving = any(velocity)
        if moving or self.base_moving:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = velocity
            self.base_pub.publish(msg)
        self.base_moving = moving
        groups = (
            ('head', ['head_joint1', 'head_joint2'],
             [('s' in keys) - ('w' in keys), ('a' in keys) - ('d' in keys)],
             .25, .05, self.head_pub),
            ('lift', ['lift_joint'], [('o' in keys) - ('p' in keys)],
             .04, .01, self.lift_pub),
        )
        for group, names, directions, speed, lead, publisher in groups:
            values = [self.measured(n) for n in names]
            if any(v is None for v in values):
                # Do not synthesize a zero pose when feedback is absent.
                self.moving.discard(group)
                self.targets.pop(group, None)
                continue
            if any(directions):
                targets = self.targets.get(group, values)
                targets = [bounded_step(v, t, d, speed, dt, LIMITS[n], lead)
                           for n, v, t, d in zip(names, values, targets, directions)]
                publisher.publish(self.trajectory(names, targets))
                self.targets[group] = targets
                self.moving.add(group)
            elif group in self.moving:
                publisher.publish(self.trajectory(names, values))
                self.moving.remove(group)
                self.targets.pop(group, None)

    def stop_motion(self):
        self.keys.clear()
        self.tick()


class KeyboardDevices:
    """Standard Linux key events, including pedals already configured as A/B/C/U."""

    def __init__(self, paths=None, loop_paths=None):
        paths = paths or sorted(glob.glob('/dev/input/by-id/*event-kbd'))
        # These are ordinary keyboard events, not a vendor pedal driver or XR UDP channel.
        loop_paths = loop_paths or [p for p in paths if 'footswitch' in p.lower()]
        loop_devices = {os.path.realpath(p) for p in loop_paths}
        paths = list(paths) + list(loop_paths)
        self.loop_fds = set()
        self.selector = selectors.DefaultSelector()
        self.buffers = {}
        self.paths = {}
        try:
            for path in sorted({os.path.realpath(p) for p in paths}):
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                self.selector.register(fd, selectors.EVENT_READ)
                self.buffers[fd], self.paths[fd] = b'', path
                if path in loop_devices:
                    self.loop_fds.add(fd)
            if not self.buffers:
                raise RuntimeError('No keyboard input device; use --device /dev/input/eventN')
        except Exception:
            self.close()
            raise

    def read(self):
        for key, _ in self.selector.select(.01):
            try:
                data = os.read(key.fd, EVENT.size * 64)
                if not data:
                    raise OSError('Keyboard disconnected')
                self.buffers[key.fd] += data
            except OSError:
                yield key.fd, -1, -1
                self.selector.unregister(key.fd)
                os.close(key.fd)
                self.buffers.pop(key.fd)
                continue
            while len(self.buffers[key.fd]) >= EVENT.size:
                raw, self.buffers[key.fd] = (self.buffers[key.fd][:EVENT.size],
                                             self.buffers[key.fd][EVENT.size:])
                _, _, kind, code, value = EVENT.unpack(raw)
                if kind == 0 and code == 3:  # SYN_DROPPED: require new physical key presses.
                    yield key.fd, -1, -1
                elif kind == 1:
                    yield key.fd, code, value

    def close(self):
        for fd in self.buffers:
            os.close(fd)
        self.buffers.clear()
        self.selector.close()


def run(node, x11, devices):
    window = x11.active_window()
    if not window:
        raise RuntimeError('Focus this local terminal before starting')
    pressed, routes = set(), {}
    focused = True

    def release_all():
        for (fd, code), (route, target) in list(routes.items()):
            if route == 'loop':
                try:
                    x11.send(target, KEYS[code], False)
                except RuntimeError as exc:
                    print(exc, flush=True)
        pressed.clear()
        routes.clear()
        node.stop_motion()

    print('U pause/resume | paused: arrows=base, Q/E=yaw, WASD=head, O/P=lift\n'
          'Pedal A/B/C=Loop; keyboard paused A=head, Shift+A=Loop A\n'
          'Space=manual stop | Esc/Ctrl+C=exit\n'
          'Only this focused terminal accepts inputs. Release keys before refocusing.', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.)
            current_focus = x11.active_window() == window
            if focused and not current_focus:
                release_all()
            focused = current_focus
            for fd, code, value in devices.read():
                if code == -1:
                    release_all()
                    print('Input lost; release keys and reconnect/restart.', flush=True)
                    continue
                item = (fd, code)
                if not focused:
                    continue
                if value == 0:
                    pressed.discard(item)
                    route, target = routes.pop(item, ('ignore', None))
                    if route == 'loop':
                        try:
                            x11.send(target, KEYS[code], False)
                        except RuntimeError as exc:
                            print(exc, flush=True)
                    continue
                if value != 1 or item in pressed:
                    continue  # OS auto-repeat never toggles U or repeats recording operations.
                pressed.add(item)
                codes = {c for _, c in pressed}
                if code == 1 or (code == 46 and codes & CTRL):
                    return
                if code == 57:
                    release_all()
                    continue
                if code not in KEYS or codes & MODIFIERS:
                    continue
                pause_context = (node.pending_command == 'pause' or bool(
                    node.status and (node.status['paused'] or node.status.get('resuming'))))
                route = key_route(KEYS[code], pause_context, bool(codes & SHIFT),
                                  fd in devices.loop_fds)
                target = None
                if route == 'toggle':
                    release_all()
                    # Keep U held so duplicate down events cannot toggle twice.
                    pressed.add(item)
                    node.toggle()
                elif route == 'loop':
                    try:
                        target = x11.loop_window()
                        if target == window:
                            raise RuntimeError('Start console with its terminal focused, not Loop')
                        x11.send(target, KEYS[code], True)
                        print(f'Loop {KEYS[code].upper()} forwarded', flush=True)
                    except RuntimeError as exc:
                        print(exc, flush=True)
                        route = 'ignore'
                routes[item] = route, target
            node.keys = {KEYS[code] for (_, code), (route, _) in routes.items()
                         if route == 'motion'}
            if not node.manual_allowed():
                routes = {item: value for item, value in routes.items() if value[0] != 'motion'}
            node.tick()
            # Drain terminal copies of the same physical events; evdev supplies real releases.
            if select.select([sys.stdin], [], [], 0)[0]:
                os.read(sys.stdin.fileno(), 4096)
    finally:
        release_all()
        node.request('pause')
        # Give reliable ROS delivery a short bounded opportunity before destroying publishers.
        deadline = time.monotonic() + .3
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        node.stop_motion()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', action='append', help='Keyboard event device; repeatable')
    parser.add_argument('--loop-device', action='append',
                        help='Keyboard device whose A/B/C always go to Loop (e.g. a pedal)')
    parser.add_argument('--check', action='store_true', help='Read-only X11 and device check')
    args, ros_args = parser.parse_known_args()
    x11 = X11Keys()
    devices = KeyboardDevices(args.device, args.loop_device)
    if args.check:
        print('Keyboard devices:', list(devices.paths.values()))
        print('Loop key devices:', [devices.paths[fd] for fd in devices.loop_fds])
        print('Active window:', x11.active_window())
        try:
            print('Loop window:', x11.loop_window())
        except RuntimeError as exc:
            print(exc)
        devices.close()
        x11.close()
        return
    if not sys.stdin.isatty():
        devices.close()
        x11.close()
        raise SystemExit('An interactive local terminal is required')
    lock = open('/tmp/quest_operator_console.lock', 'w')
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit('An operator console is already running in this container')
    old = termios.tcgetattr(sys.stdin.fileno())
    rclpy.init(args=ros_args)
    node = OperatorConsole()
    try:
        tty.setraw(sys.stdin.fileno())
        # Keep normal newline rendering while suppressing local key echo/signals.
        settings = termios.tcgetattr(sys.stdin.fileno())
        settings[1] |= termios.OPOST
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, settings)
        run(node, x11, devices)
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)
        devices.close()
        x11.close()
        node.destroy_node()
        lock.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
