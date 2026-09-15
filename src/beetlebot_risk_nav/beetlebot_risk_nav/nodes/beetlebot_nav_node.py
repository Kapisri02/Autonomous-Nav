#!/usr/bin/env python3
"""ROS 2 node: autonomous start-to-goal navigation for the BeetleBot.

Thin adapter around :class:`beetlebot_risk_nav.core.navigator.Navigator`. All
decision-making lives in the ROS-free core; this file only converts messages,
looks up transforms and publishes commands, which is why the core can be tested
without ROS at all.

Interfaces
----------
Subscribes
    ``/scan``       ``sensor_msgs/LaserScan``  - RPLiDAR C1
    ``/odom``       ``nav_msgs/Odometry``      - wheel/EKF odometry
    ``/goal_pose``  ``geometry_msgs/PoseStamped`` - RViz "2D Goal Pose"
Publishes
    ``/cmd_vel_nav``            ``geometry_msgs/Twist``  (topic is a parameter)
    ``~/status``                ``std_msgs/String``      - human-readable state
    ``~/goal_reached``          ``std_msgs/Bool``        - latched SUCCESS flag

Pose source
-----------
The goal arrives in the map frame from RViz, so the robot's pose is taken from
TF (``global_frame`` -> ``base_frame``), which is what the existing localisation
publishes. If that transform is unavailable the node falls back to raw odometry
and says so: odometry alone still navigates correctly relative to where the
robot started, it just cannot honour a goal expressed on the map.

Stopping
--------
On shutdown the node publishes repeated zero-velocity commands, the same
protection the baseline controller implements, so that Ctrl+C never leaves the
wheels turning.
"""

from __future__ import annotations

import math
import sys
import time
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from beetlebot_risk_nav.core.geometry import yaw_from_quaternion
from beetlebot_risk_nav.core.logging_utils import NavigationLogger
from beetlebot_risk_nav.core.navigator import STATE_GOAL_REACHED, Navigator
from beetlebot_risk_nav.core.params import NavConfig, describe
from beetlebot_risk_nav.core.types import ScanData

try:                                    # tf2 is optional: without it we use odometry
    import tf2_ros
    from rclpy.duration import Duration
    TF2_AVAILABLE = True
except ImportError:                     # pragma: no cover - depends on the install
    TF2_AVAILABLE = False


SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class BeetleBotNavNode(Node):

    def __init__(self) -> None:
        super().__init__('beetlebot_nav')

        # --- parameters ---------------------------------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        # Default matches the proven baseline, which publishes to /cmd_vel_nav
        # and lets the existing cmd_vel_mux arbitrate.
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('use_tf', True)
        self.declare_parameter('publish_status', True)

        self.config = NavConfig()
        self._declare_config_parameters()

        problems = self.config.validate()
        if problems:
            for problem in problems:
                self.get_logger().error('invalid configuration: {}'.format(problem))
            raise RuntimeError('refusing to start with an invalid configuration')

        self.navigator = Navigator(self.config)
        self.logger = NavigationLogger(self.config.logging)

        # --- state ---------------------------------------------------------
        self._scan: Optional[ScanData] = None
        self._scan_stamp: Optional[float] = None
        self._odom_pose: Optional[Tuple[float, float, float]] = None
        self._odom_stamp: Optional[float] = None
        self._velocity: Tuple[float, float] = (0.0, 0.0)
        self._goal_announced = False
        self._warned_no_tf = False
        self._overrun_count = 0

        # --- interfaces -------------------------------------------------------
        scan_topic = self.get_parameter('scan_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        goal_topic = self.get_parameter('goal_topic').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value

        self.create_subscription(LaserScan, scan_topic, self._on_scan, SENSOR_QOS)
        self.create_subscription(Odometry, odom_topic, self._on_odom, SENSOR_QOS)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.reached_pub = self.create_publisher(Bool, '~/goal_reached', 10)

        self.tf_buffer = None
        self.tf_listener = None
        if TF2_AVAILABLE and self.get_parameter('use_tf').value:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        period = 1.0 / max(1e-3, float(self.get_parameter('control_frequency').value))
        self._period = period
        self.create_timer(period, self._control_cycle)

        self.get_logger().info(
            'BeetleBot navigation started\n'
            '  scan:    {}\n  odom:    {}\n  goal:    {}\n  cmd_vel: {}\n'
            '  frames:  {} -> {} (tf {})\n  rate:    {:.1f} Hz\n'
            '  footprint: {:.3f} x {:.3f} m ({}), inflation {:.3f} m'.format(
                scan_topic, odom_topic, goal_topic, cmd_topic,
                self.get_parameter('global_frame').value,
                self.get_parameter('base_frame').value,
                'on' if self.tf_buffer else 'off',
                1.0 / period,
                self.config.robot.footprint_length, self.config.robot.footprint_width,
                'circular' if self.config.robot.use_circular_footprint else 'rectangular',
                self.config.robot.footprint_inflation))
        self.get_logger().debug(describe(self.config))
        self.get_logger().info('waiting for a goal on {} (RViz "2D Goal Pose")'.format(
            goal_topic))

    # ------------------------------------------------------------------
    def _declare_config_parameters(self) -> None:
        """Expose every tunable as a ROS parameter using its dotted name."""
        for key, value in self.config.to_flat().items():
            self.declare_parameter(key, value)
        overrides = {}
        for key in self.config.to_flat():
            overrides[key] = self.get_parameter(key).value
        unknown = self.config.apply_flat(overrides)
        for key in unknown:                     # pragma: no cover - defensive
            self.get_logger().warn('ignored unknown parameter {}'.format(key))

    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        stamp = _stamp_seconds(msg.header.stamp) or self._now()
        self._scan = ScanData(
            stamp=stamp,
            angle_min=float(msg.angle_min),
            angle_increment=float(msg.angle_increment),
            ranges=list(msg.ranges),
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
            frame_id=msg.header.frame_id,
        )
        self._scan_stamp = stamp

    def _on_odom(self, msg: Odometry) -> None:
        stamp = _stamp_seconds(msg.header.stamp) or self._now()
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self._odom_pose = (position.x, position.y,
                           yaw_from_quaternion(orientation.x, orientation.y,
                                               orientation.z, orientation.w))
        self._odom_stamp = stamp
        self._velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_goal(self, msg: PoseStamped) -> None:
        goal = self._goal_in_working_frame(msg)
        if goal is None:
            self.get_logger().error(
                'cannot use goal in frame "{}": no transform to {}'.format(
                    msg.header.frame_id, self.get_parameter('global_frame').value))
            return
        self.navigator.set_goal(goal, self._now())
        path = self.logger.start_episode(goal, time.time())
        if path:
            self.get_logger().info('logging this run to {}'.format(path))
        elif self.logger.disabled_reason:
            self.get_logger().warn('logging disabled: {}'.format(self.logger.disabled_reason))
        self._goal_announced = False
        self.reached_pub.publish(Bool(data=False))
        self.get_logger().info('new goal: ({:.2f}, {:.2f})'.format(*goal))

    # ------------------------------------------------------------------
    def _control_cycle(self) -> None:
        now = self._now()
        pose = self._robot_pose()
        if pose is None:
            self._publish(Twist())
            self._publish_status('waiting for a robot pose (TF or odometry)')
            return

        result = self.navigator.update(self._scan, pose, self._velocity, now,
                                       odom_stamp=self._odom_stamp)

        twist = Twist()
        twist.linear.x = float(result.command.v)
        twist.angular.z = float(result.command.w)
        self._publish(twist)
        self._publish_status(result.status)
        if not result.finished:
            self.logger.log_cycle(now, result, pose)

        if result.state == STATE_GOAL_REACHED and not self._goal_announced:
            self._goal_announced = True
            self.reached_pub.publish(Bool(data=True))
            self.logger.log_cycle(now, result, pose)
            self.logger.close()
            self.get_logger().info('SUCCESS: {}'.format(result.status))
        elif result.state == 'FAILED' and not self._goal_announced:
            self._goal_announced = True
            self.logger.log_cycle(now, result, pose)
            self.logger.close()
            self.get_logger().error('navigation failed: {}'.format(result.status))

        if result.plan is not None:
            elapsed = result.plan.debug.compute_time
            if elapsed > self._period:
                self._overrun_count += 1
                if self._overrun_count % 10 == 1:
                    self.get_logger().warn(
                        'control cycle took {:.1f} ms, over the {:.1f} ms budget '
                        '({} overruns); consider lidar.decimation'.format(
                            elapsed * 1000.0, self._period * 1000.0, self._overrun_count))

    # ------------------------------------------------------------------
    def _robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Robot pose in the working frame: TF if available, else odometry."""
        if self.tf_buffer is not None:
            global_frame = self.get_parameter('global_frame').value
            base_frame = self.get_parameter('base_frame').value
            try:
                tf = self.tf_buffer.lookup_transform(
                    global_frame, base_frame, rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
                t = tf.transform.translation
                r = tf.transform.rotation
                return (t.x, t.y, yaw_from_quaternion(r.x, r.y, r.z, r.w))
            except Exception as exc:            # tf2 raises several exception types
                if not self._warned_no_tf:
                    self._warned_no_tf = True
                    self.get_logger().warn(
                        '{} -> {} transform unavailable ({}); falling back to raw '
                        'odometry. Goals set on the map will not be honoured until '
                        'localisation is running.'.format(global_frame, base_frame, exc))
        return self._odom_pose

    def _goal_in_working_frame(self, msg: PoseStamped) -> Optional[Tuple[float, float]]:
        global_frame = self.get_parameter('global_frame').value
        frame = msg.header.frame_id or global_frame
        point = (msg.pose.position.x, msg.pose.position.y)
        if frame == global_frame:
            return point
        if self.tf_buffer is None:
            # Without TF there is no way to resolve another frame. Accepting the
            # coordinates anyway would drive confidently to the wrong place,
            # which is worse than refusing the goal.
            return None
        try:
            tf = self.tf_buffer.lookup_transform(global_frame, frame, rclpy.time.Time(),
                                                 timeout=Duration(seconds=0.2))
            t = tf.transform.translation
            r = tf.transform.rotation
            yaw = yaw_from_quaternion(r.x, r.y, r.z, r.w)
            return (t.x + point[0] * math.cos(yaw) - point[1] * math.sin(yaw),
                    t.y + point[0] * math.sin(yaw) + point[1] * math.cos(yaw))
        except Exception:
            return None

    # ------------------------------------------------------------------
    def _publish(self, twist: Twist) -> None:
        self.cmd_pub.publish(twist)

    def _publish_status(self, text: str) -> None:
        if self.get_parameter('publish_status').value:
            self.status_pub.publish(String(data=text))

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def halt(self, repeats: int = 10) -> None:
        """Publish repeated stops so shutdown can never leave the wheels turning."""
        stop = Twist()
        for _ in range(repeats):
            try:
                self.cmd_pub.publish(stop)
            except Exception:                   # pragma: no cover - during teardown
                break


def _stamp_seconds(stamp) -> Optional[float]:
    try:
        value = stamp.sec + stamp.nanosec * 1e-9
    except AttributeError:                      # pragma: no cover - defensive
        return None
    return value if value > 0.0 else None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = BeetleBotNavNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:                    # pragma: no cover - startup failures
        print('beetlebot_nav failed: {}'.format(exc), file=sys.stderr)
    finally:
        if node is not None:
            node.get_logger().info('stopping the robot before shutdown')
            node.halt()
            node.logger.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
