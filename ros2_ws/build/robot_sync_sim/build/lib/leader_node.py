#!/usr/bin/env python3
"""Leader robot: drive forward, spawn trash items behind the robot at a distance interval."""

import math
import threading

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import PoseStamped, Twist
from gazebo_msgs.srv import SpawnEntity
from nav_msgs.msg import Odometry


def _yaw_from_quat(q) -> float:
    s = 2.0 * (q.w * q.z + q.x * q.y)
    c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(s, c)


class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')

        self.declare_parameter('linear_speed', 1.2)
        self.declare_parameter('drop_interval_m', 2.0)
        # With spawn yaw=0, base +X aligns with world +X; sign is usually +1.0
        self.declare_parameter('cmd_vel_forward_sign', 1.0)
        self.declare_parameter('drop_behind_m', 0.32)
        # Use full mode name; bare "y" in YAML parses as boolean
        self.declare_parameter('progress_mode', 'full_path')
        self.declare_parameter('odometry_warmup_samples', 15)
        self.declare_parameter('cmd_vel_topic', '/robot_a/cmd_vel')
        self.declare_parameter('odom_topic', '/robot_a/odom')
        self.declare_parameter('max_drops', 2)

        self._v = float(self.get_parameter('linear_speed').value)
        self._cmd_sign = float(self.get_parameter('cmd_vel_forward_sign').value)
        self._interval = float(self.get_parameter('drop_interval_m').value)
        self._behind = float(self.get_parameter('drop_behind_m').value)
        self._mode = str(self.get_parameter('progress_mode').value).strip().lower()
        if self._mode not in ('along_y', 'full_path'):
            self._mode = 'along_y'
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        # Reliable cmd_vel matches gazebo plugin subscription (avoid system_default depth=0)
        self._cmd_vel_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub_vel = self.create_publisher(Twist, cmd_topic, self._cmd_vel_qos)
        self._pub_trash = self.create_publisher(PoseStamped, '/trash_points', 10)

        self._spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self._trash_xml_template = self._load_trash_template()
        self._trash_id = 0
        self._lock = threading.Lock()

        self._last_x = None
        self._last_y = None
        self._accum = 0.0
        self._warmup_target = int(self.get_parameter('odometry_warmup_samples').value)
        self._warmup_count = 0
        # First sample after warmup only syncs pose (avoids drop on odom jump)
        self._odom_settled = False
        self._max_drops = int(self.get_parameter('max_drops').value)
        self._drops_done = 0
        self._mission_done = False

        self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data)

        # Timer keeps cmd_vel even if odom QoS mismatches
        self.create_timer(0.02, self._publish_cmd_vel)

        self.get_logger().info(
            f'Leader: v={self._v} m/s, drop every {self._interval} m '
            f'({self._mode}), behind={self._behind} m, max_drops={self._max_drops}')

    def _load_trash_template(self) -> str:
        try:
            share = get_package_share_directory('robot_sync_sim')
        except LookupError as e:
            self.get_logger().fatal(f'Package robot_sync_sim not found: {e}')
            raise
        path = f'{share}/models/trash/model.sdf'
        try:
            with open(path, encoding='utf-8') as f:
                return f.read()
        except OSError as e:
            self.get_logger().fatal(f'Cannot read trash model: {path} ({e})')
            raise

    def _on_odom(self, msg: Odometry):
        if self._mission_done:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)

        with self._lock:
            if self._drops_done >= self._max_drops:
                return
            if self._warmup_count < self._warmup_target:
                self._last_x, self._last_y = x, y
                self._warmup_count += 1
                self._accum = 0.0
                return

            if not self._odom_settled:
                self._last_x, self._last_y = x, y
                self._accum = 0.0
                self._odom_settled = True
                return

            if self._mode == 'along_y':
                self._accum += abs(y - self._last_y)
            else:
                self._accum += math.hypot(x - self._last_x, y - self._last_y)

            self._last_x, self._last_y = x, y

            if self._accum < self._interval:
                return
            self._accum = 0.0

        # Drop pose: behind the robot, opposite travel direction
        bx = x - math.cos(yaw) * self._behind
        by = y - math.sin(yaw) * self._behind

        self._spawn_trash_and_publish(bx, by)

    def _spawn_trash_and_publish(self, wx: float, wy: float):
        from geometry_msgs.msg import Pose

        self._trash_id += 1
        name = f'trash_{self._trash_id}'
        xml = self._trash_xml_template.replace('trash_item', name)

        pose = Pose()
        pose.position.x = wx
        pose.position.y = wy
        pose.position.z = 0.08
        pose.orientation.w = 1.0

        if not self._spawn_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('spawn_entity service not available')
            return

        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.robot_namespace = ''
        req.initial_pose = pose
        req.reference_frame = 'world'

        try:
            fut = self._spawn_cli.call_async(req)
        except Exception as e:
            self.get_logger().error(f'spawn_entity async failed: {e}')
            return

        def _done(f):
            try:
                res = f.result()
            except Exception as e:
                self.get_logger().error(f'spawn_entity exception: {e}')
                return
            if not res.success:
                self.get_logger().error(f'spawn failed: {res.status_message}')
                return
            self.get_logger().info(f'Spawned {name} behind robot at ({wx:.3f}, {wy:.3f})')
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'world'
            ps.pose = pose
            self._pub_trash.publish(ps)
            with self._lock:
                self._drops_done += 1
                if self._drops_done >= self._max_drops:
                    self._mission_done = True
                    self.get_logger().info(
                        f'Leader: finished {self._max_drops} drops, stopping.')

        fut.add_done_callback(_done)

    def _publish_cmd_vel(self):
        t = Twist()
        if self._mission_done:
            self._pub_vel.publish(t)
            return
        t.linear.x = self._cmd_sign * abs(self._v)
        self._pub_vel.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
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
