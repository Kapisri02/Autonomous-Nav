"""Minimal stand-ins for the ROS 2 Python API.

They exist so that the node's own logic - message conversion, parameter
declaration, topic wiring, the control cycle and the shutdown stop - can be
executed and asserted on in an environment with no ROS installed.

This tests OUR code, not ROS. Anything that depends on real middleware
behaviour (QoS negotiation, TF, timing, message transport) is untested here and
remains unverified until the package runs on the robot.
"""

from __future__ import annotations

import sys
import types
from typing import Any, Dict, List


class _Msg:
    """Recursive attribute bag that mimics a generated ROS message."""

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


def _vec3(x=0.0, y=0.0, z=0.0):
    return _Msg(x=x, y=y, z=z)


def _quat(x=0.0, y=0.0, z=0.0, w=1.0):
    return _Msg(x=x, y=y, z=z, w=w)


def _stamp(sec=0, nanosec=0):
    return _Msg(sec=sec, nanosec=nanosec)


def _header(frame_id='', sec=0, nanosec=0):
    return _Msg(frame_id=frame_id, stamp=_stamp(sec, nanosec))


class Twist(_Msg):
    def __init__(self, **kwargs):
        super().__init__(linear=_vec3(), angular=_vec3(), **kwargs)


class PoseStamped(_Msg):
    def __init__(self, **kwargs):
        super().__init__(header=_header(),
                         pose=_Msg(position=_vec3(), orientation=_quat()), **kwargs)


class Odometry(_Msg):
    def __init__(self, **kwargs):
        super().__init__(
            header=_header(),
            pose=_Msg(pose=_Msg(position=_vec3(), orientation=_quat())),
            twist=_Msg(twist=_Msg(linear=_vec3(), angular=_vec3())), **kwargs)


class LaserScan(_Msg):
    def __init__(self, **kwargs):
        super().__init__(header=_header(), angle_min=0.0, angle_max=0.0,
                         angle_increment=0.0, range_min=0.0, range_max=0.0,
                         ranges=[], **kwargs)


class String(_Msg):
    def __init__(self, data=''):
        super().__init__(data=data)


class Bool(_Msg):
    def __init__(self, data=False):
        super().__init__(data=data)


class FakePublisher:
    def __init__(self, topic: str):
        self.topic = topic
        self.messages: List[Any] = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeLogger:
    def __init__(self):
        self.records: Dict[str, List[str]] = {}

    def _log(self, level, text):
        self.records.setdefault(level, []).append(str(text))

    def info(self, text):
        self._log('info', text)

    def warn(self, text):
        self._log('warn', text)

    def error(self, text):
        self._log('error', text)

    def debug(self, text):
        self._log('debug', text)


class FakeClock:
    def __init__(self):
        self.seconds = 0.0

    def now(self):
        return _Msg(nanoseconds=int(self.seconds * 1e9))


class FakeNode:
    """Enough of rclpy.node.Node for the navigation node to run."""

    def __init__(self, name: str):
        self._name = name
        self._parameters: Dict[str, Any] = {}
        self.subscriptions_by_topic: Dict[str, Any] = {}
        self.publishers_by_topic: Dict[str, FakePublisher] = {}
        self.timers: List[Any] = []
        self._logger = FakeLogger()
        self._clock = FakeClock()

    def declare_parameter(self, name, value=None):
        self._parameters.setdefault(name, value)
        return _Msg(value=self._parameters[name])

    def get_parameter(self, name):
        return _Msg(value=self._parameters[name])

    def set_parameter_value(self, name, value):
        self._parameters[name] = value

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions_by_topic[topic] = callback
        return _Msg(topic=topic)

    def create_publisher(self, msg_type, topic, qos):
        publisher = FakePublisher(topic)
        self.publishers_by_topic[topic] = publisher
        return publisher

    def create_timer(self, period, callback):
        timer = _Msg(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def destroy_node(self):
        pass


def install() -> None:
    """Register the stub modules in ``sys.modules``.

    tf2_ros is deliberately NOT provided, so the node takes its documented
    odometry fallback path - which is the path that runs when localisation is
    not up, and therefore the one worth testing.
    """
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.try_shutdown = lambda *a, **k: None

    time_mod = types.ModuleType('rclpy.time')
    time_mod.Time = lambda *a, **k: _Msg()
    rclpy.time = time_mod

    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = FakeNode

    qos_mod = types.ModuleType('rclpy.qos')
    qos_mod.QoSProfile = lambda **kwargs: _Msg(**kwargs)
    qos_mod.QoSReliabilityPolicy = _Msg(BEST_EFFORT=1, RELIABLE=0)
    qos_mod.QoSHistoryPolicy = _Msg(KEEP_LAST=1)
    qos_mod.QoSDurabilityPolicy = _Msg(VOLATILE=2, TRANSIENT_LOCAL=1)

    duration_mod = types.ModuleType('rclpy.duration')
    duration_mod.Duration = lambda **kwargs: _Msg(**kwargs)

    geometry = types.ModuleType('geometry_msgs')
    geometry_msg = types.ModuleType('geometry_msgs.msg')
    geometry_msg.Twist = Twist
    geometry_msg.PoseStamped = PoseStamped
    geometry.msg = geometry_msg

    nav = types.ModuleType('nav_msgs')
    nav_msg = types.ModuleType('nav_msgs.msg')
    nav_msg.Odometry = Odometry
    nav.msg = nav_msg

    sensor = types.ModuleType('sensor_msgs')
    sensor_msg = types.ModuleType('sensor_msgs.msg')
    sensor_msg.LaserScan = LaserScan
    sensor.msg = sensor_msg

    std = types.ModuleType('std_msgs')
    std_msg = types.ModuleType('std_msgs.msg')
    std_msg.String = String
    std_msg.Bool = Bool
    std.msg = std_msg

    sys.modules.update({
        'rclpy': rclpy,
        'rclpy.time': time_mod,
        'rclpy.node': node_mod,
        'rclpy.qos': qos_mod,
        'rclpy.duration': duration_mod,
        'geometry_msgs': geometry,
        'geometry_msgs.msg': geometry_msg,
        'nav_msgs': nav,
        'nav_msgs.msg': nav_msg,
        'sensor_msgs': sensor,
        'sensor_msgs.msg': sensor_msg,
        'std_msgs': std,
        'std_msgs.msg': std_msg,
    })
