#!/usr/bin/env python3
"""Follower: drive to trash, enable vacuum while stopped, wait for grasp (no forward shove)."""

import math
import threading
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


def _wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_from_quat(q) -> float:
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


class FollowerNode(Node):
    def __init__(self):
        super().__init__('follower_node')

        # Leader drives +X; follower starts behind on negative X (not Y).
        self.declare_parameter('b_spawn_x', -1.0)
        self.declare_parameter('b_spawn_y', 0.0)
        self.declare_parameter('goal_tolerance_m', 0.12)
        # Forward offset from base_link origin to vacuum plate (robot_b gripper joint)
        self.declare_parameter('vacuum_forward_offset_m', 0.14)
        self.declare_parameter('kp_linear', 2.8)
        self.declare_parameter('kp_angular', 3.0)
        self.declare_parameter('max_linear', 1.5)
        self.declare_parameter('max_angular', 2.0)
        self.declare_parameter('align_threshold_rad', 0.35)
        # Matches gazebo_ros_vacuum_gripper log: "Advertise gripper status on [/robot_b/grasping]"
        self.declare_parameter('grasping_topic', '/robot_b/grasping')
        self.declare_parameter('grasp_timeout_s', 5.0)
        self.declare_parameter('post_grasp_hold_s', 0.5)
        self.declare_parameter('cmd_vel_topic', '/robot_b/cmd_vel')
        self.declare_parameter('odom_topic', '/robot_b/odom')
        self.declare_parameter('vacuum_service', '/robot_b/activate_vacuum')
        self.declare_parameter('cmd_vel_forward_sign', 1.0)
        self.declare_parameter('max_pickups', 2)
        self.declare_parameter('post_mission_vacuum_duration_s', 5.0)

        self._bx0 = float(self.get_parameter('b_spawn_x').value)
        self._by0 = float(self.get_parameter('b_spawn_y').value)
        self._tol = float(self.get_parameter('goal_tolerance_m').value)
        self._kp_lin = float(self.get_parameter('kp_linear').value)
        self._kp_ang = float(self.get_parameter('kp_angular').value)
        self._v_max = float(self.get_parameter('max_linear').value)
        self._w_max = float(self.get_parameter('max_angular').value)
        self._align_thr = float(self.get_parameter('align_threshold_rad').value)
        self._grasp_topic = self.get_parameter('grasping_topic').value
        self._grasp_timeout = float(self.get_parameter('grasp_timeout_s').value)
        self._post_grasp_hold = float(self.get_parameter('post_grasp_hold_s').value)
        self._vacuum_off = float(self.get_parameter('vacuum_forward_offset_m').value)
        self._cmd_sign = float(self.get_parameter('cmd_vel_forward_sign').value)
        self._max_pickups = int(self.get_parameter('max_pickups').value)
        self._post_mission_vacuum_dur = float(
            self.get_parameter('post_mission_vacuum_duration_s').value)
        self._pickups_done = 0
        self._mission_done = False
        self._post_mission_vacuum_until = None

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        vac_srv = self.get_parameter('vacuum_service').value

        _cmd_vel_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(Twist, cmd_topic, _cmd_vel_qos)
        self._vacuum_cli = self.create_client(SetBool, vac_srv)

        self._queue = deque()
        self._q_lock = threading.Lock()
        self._active_goal = None
        self._state = 'idle'
        self._grasp_deadline = None
        self._hold_end_time = None
        self._grasping = False

        self._odom_rx = 0.0
        self._odom_ry = 0.0
        self._odom_yaw = 0.0

        self.create_subscription(
            PoseStamped, '/trash_points', self._on_trash, 10)
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            Bool, self._grasp_topic, self._on_grasping, 10)

        self.get_logger().info(
            f'Follower: spawn ({self._bx0}, {self._by0}) tol={self._tol} m, '
            f'cmd_sign={self._cmd_sign}, max_pickups={self._max_pickups}')

    def _on_trash(self, msg: PoseStamped):
        if self._mission_done:
            return
        p = msg.pose.position
        with self._q_lock:
            self._queue.append((float(p.x), float(p.y)))
        self.get_logger().info(f'Queued trash goal ({p.x:.3f}, {p.y:.3f})')

    def _on_grasping(self, msg: Bool):
        self._grasping = bool(msg.data)

    def _on_odom(self, msg: Odometry):
        self._odom_rx = msg.pose.pose.position.x
        self._odom_ry = msg.pose.pose.position.y
        self._odom_yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._control_loop()

    def _error_to_goal(self, tx: float, ty: float):
        # gazebo_ros_planar_move fills pose from WorldPose(): (x,y) are world-frame.
        # Do not subtract spawn from goal separately from odom.
        ex = tx - self._odom_rx
        ey = ty - self._odom_ry
        return ex, ey

    def _nav_goal_for_vacuum_over_trash(self, tx: float, ty: float):
        """Base-link goal: offset back from trash along approach so the vacuum sits on the cube."""
        ex = tx - self._odom_rx
        ey = ty - self._odom_ry
        dist = math.hypot(ex, ey)
        if dist < 1e-4:
            return tx, ty
        ux, uy = ex / dist, ey / dist
        return tx - self._vacuum_off * ux, ty - self._vacuum_off * uy

    def _control_loop(self):
        twist = Twist()
        now = self.get_clock().now()

        if self._post_mission_vacuum_until is not None:
            self._pub.publish(Twist())
            if now >= self._post_mission_vacuum_until:
                self._post_mission_vacuum_until = None
                self._vacuum_off_post_mission()
            return

        if self._mission_done:
            self._pub.publish(twist)
            return

        with self._q_lock:
            if self._active_goal is None and self._queue and self._state == 'idle':
                self._active_goal = self._queue.popleft()
                self._state = 'navigate'
                self.get_logger().info(f'Active goal {self._active_goal}')

        if self._active_goal is None:
            self._pub.publish(twist)
            return

        if self._state == 'pickup_wait':
            self._pub.publish(Twist())
            return

        if self._state == 'pickup_wait_grasp':
            self._pub.publish(Twist())
            if self._grasping:
                self._hold_end_time = now + Duration(seconds=self._post_grasp_hold)
                self._state = 'post_grasp_hold'
                self.get_logger().info('Vacuum grasp detected, holding.')
                return
            if self._grasp_deadline is not None and now > self._grasp_deadline:
                self.get_logger().warn(
                    'Grasp timeout: vacuum did not report grasp; releasing.')
                self._vacuum_off_and_finish()
            return

        if self._state == 'post_grasp_hold':
            self._pub.publish(Twist())
            if self._hold_end_time is not None and now >= self._hold_end_time:
                if self._pickups_done == self._max_pickups - 1:
                    self._finish_last_pickup_keep_vacuum(now)
                else:
                    self._vacuum_off_and_finish()
            return

        tx, ty = self._active_goal
        ngx, ngy = self._nav_goal_for_vacuum_over_trash(tx, ty)
        ex, ey = self._error_to_goal(ngx, ngy)
        dist = math.hypot(ex, ey)
        heading_goal = math.atan2(ey, ex)
        heading_err = _wrap_angle(heading_goal - self._odom_yaw)

        if self._state == 'navigate':
            if dist <= self._tol:
                self._pub.publish(Twist())
                self._vacuum_on_and_prepare_pickup()
                return

            # Unicycle: forward speed ~ dist * cos(heading_err); turn ~ heading_err
            v_raw = self._kp_lin * dist * math.cos(heading_err)
            omega = self._kp_ang * heading_err

            if abs(heading_err) > self._align_thr:
                v_raw = 0.0

            twist.linear.x = self._cmd_sign * max(
                -self._v_max, min(self._v_max, v_raw))
            twist.angular.z = max(
                -self._w_max, min(self._w_max, omega))
            self._pub.publish(twist)

    def _vacuum_on_and_prepare_pickup(self):
        self._state = 'pickup_wait'
        if not self._vacuum_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Vacuum service unavailable')
            self._clear_goal()
            return

        req = SetBool.Request(data=True)
        fut = self._vacuum_cli.call_async(req)

        def _done(f):
            try:
                res = f.result()
            except Exception as e:
                self.get_logger().error(f'Vacuum on: {e}')
                self._clear_goal()
                return
            if not res.success:
                self.get_logger().warn('Vacuum on: success=false')
            self._grasp_deadline = self.get_clock().now() + Duration(
                seconds=self._grasp_timeout)
            self._state = 'pickup_wait_grasp'

        fut.add_done_callback(_done)

    def _finish_last_pickup_keep_vacuum(self, now):
        """Last pickup done: mission complete but leave vacuum on for a few seconds."""
        self._state = 'idle'
        self._grasp_deadline = None
        self._hold_end_time = None
        self._active_goal = None
        self._pickups_done += 1
        self._post_mission_vacuum_until = now + Duration(
            seconds=self._post_mission_vacuum_dur)
        self._mission_done = True
        self.get_logger().info(
            f'Follower: mission done ({self._pickups_done} pickups). '
            f'Vacuum stays on {self._post_mission_vacuum_dur:.1f} s.')

    def _vacuum_off_post_mission(self):
        if not self._vacuum_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Vacuum off (post-mission): service unavailable')
            return
        req = SetBool.Request(data=False)
        fut = self._vacuum_cli.call_async(req)

        def _done(f):
            try:
                f.result()
            except Exception as e:
                self.get_logger().warn(f'Vacuum off (post-mission): {e}')
            self.get_logger().info('Vacuum off after post-mission delay.')

        fut.add_done_callback(_done)

    def _vacuum_off_and_finish(self):
        self._state = 'idle'
        self._grasp_deadline = None
        self._hold_end_time = None

        def _finish():
            self._active_goal = None
            self._pickups_done += 1
            self.get_logger().info(
                f'Pickup complete ({self._pickups_done}/{self._max_pickups})')
            if self._pickups_done >= self._max_pickups:
                self._mission_done = True
                self.get_logger().info('Follower: mission done, stopping.')

        if not self._vacuum_cli.wait_for_service(timeout_sec=1.0):
            _finish()
            return
        req = SetBool.Request(data=False)
        fut = self._vacuum_cli.call_async(req)

        def _done(f):
            try:
                f.result()
            except Exception as e:
                self.get_logger().warn(f'Vacuum off: {e}')
            _finish()

        fut.add_done_callback(_done)

    def _clear_goal(self):
        self._active_goal = None
        self._state = 'idle'
        self._grasp_deadline = None
        self._hold_end_time = None


def main(args=None):
    rclpy.init(args=args)
    node = FollowerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
