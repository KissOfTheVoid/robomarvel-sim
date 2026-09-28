"""sim_lidar - функциональная замена драйвера лидара.

Публикует /scan (sensor_msgs/LaserScan) в системе координат `lidar`,
raycast'ом по виртуальной планировке из world.yaml.

Смещение лидара относительно base_link берётся из TF (robot_state_publisher
поднимает его из robot.urdf), а не хардкодится - так симуляция не может
разъехаться с URDF робота.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from robomarvel_sim.world import World


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimLidar(Node):
    def __init__(self):
        super().__init__("sim_lidar")

        self.declare_parameter("world", "")
        self.declare_parameter("frame_id", "lidar")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("num_rays", 450)
        self.declare_parameter("angle_min", 0.0)
        self.declare_parameter("angle_max", 2.0 * math.pi)
        self.declare_parameter("range_min", 0.03)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("noise_std", 0.02)
        self.declare_parameter("dropout", 0.02)
        self.declare_parameter("publish_intensities", True)
        # запасные значения, если TF ещё не поднялся (совпадают с robot.urdf)
        self.declare_parameter("fallback_offset_x", 0.06)
        self.declare_parameter("fallback_offset_y", 0.0)
        self.declare_parameter("fallback_offset_yaw", 1.57)

        world_path = self.get_parameter("world").value
        if not world_path:
            raise RuntimeError("Параметр 'world' обязателен для sim_lidar")
        self.world = World.from_yaml(world_path)

        self.frame_id = self.get_parameter("frame_id").value
        self.base_frame = self.get_parameter("base_frame").value
        self.rate = float(self.get_parameter("rate").value)
        self.n = int(self.get_parameter("num_rays").value)
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.noise_std = float(self.get_parameter("noise_std").value)
        self.dropout = float(self.get_parameter("dropout").value)
        self.with_intensities = bool(self.get_parameter("publish_intensities").value)

        self.angle_increment = (self.angle_max - self.angle_min) / self.n
        self.local_angles = self.angle_min + np.arange(self.n) * self.angle_increment

        self.off_x = float(self.get_parameter("fallback_offset_x").value)
        self.off_y = float(self.get_parameter("fallback_offset_y").value)
        self.off_yaw = float(self.get_parameter("fallback_offset_yaw").value)
        self.offset_from_tf = False

        self.pose = None  # (x, y, yaw) истинная поза base_link

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # /scan публикуем RELIABLE: такой издатель совместим и с RELIABLE-подписчиками
        # (rtabmap), и с BEST_EFFORT (costmap'ы Nav2). Обратное - не работает.
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos)
        self.truth_sub = self.create_subscription(
            Odometry, "/sim/ground_truth", self.truth_callback, 5
        )

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f"sim_lidar: мир '{self.world.name}', {self.n} лучей, "
            f"{self.rate:.0f} Гц, система координат '{self.frame_id}'"
        )

    def truth_callback(self, msg: Odometry):
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quat_to_yaw(msg.pose.pose.orientation),
        )

    def try_offset_from_tf(self):
        if self.offset_from_tf:
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.frame_id, Time())
        except Exception:
            return
        self.off_x = tf.transform.translation.x
        self.off_y = tf.transform.translation.y
        self.off_yaw = quat_to_yaw(tf.transform.rotation)
        self.offset_from_tf = True
        self.get_logger().info(
            f"Смещение {self.base_frame}->{self.frame_id} взято из TF: "
            f"x={self.off_x:.3f} y={self.off_y:.3f} yaw={self.off_yaw:.3f}"
        )

    def tick(self):
        if self.pose is None:
            return
        self.try_offset_from_tf()

        bx, by, byaw = self.pose
        cos_b, sin_b = math.cos(byaw), math.sin(byaw)
        lx = bx + cos_b * self.off_x - sin_b * self.off_y
        ly = by + sin_b * self.off_x + cos_b * self.off_y
        lyaw = byaw + self.off_yaw

        ranges = self.world.raycast(lx, ly, lyaw + self.local_angles)

        if self.noise_std > 0.0:
            ranges = ranges + np.random.normal(0.0, self.noise_std, ranges.shape)
        if self.dropout > 0.0:
            lost = np.random.random(ranges.shape) < self.dropout
            ranges = np.where(lost, np.inf, ranges)

        # вне диапазона -> inf, как у настоящего драйвера
        ranges = np.where(
            (ranges < self.range_min) | (ranges > self.range_max), np.inf, ranges
        )

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = float(self.angle_min)
        msg.angle_max = float(self.angle_max - self.angle_increment)
        msg.angle_increment = float(self.angle_increment)
        msg.time_increment = float(1.0 / self.rate / self.n)
        msg.scan_time = float(1.0 / self.rate)
        msg.range_min = float(self.range_min)
        msg.range_max = float(self.range_max)
        msg.ranges = [float(r) for r in ranges]
        if self.with_intensities:
            finite = np.isfinite(ranges)
            inten = np.where(finite, 200.0, 0.0)
            msg.intensities = [float(i) for i in inten]
        self.scan_pub.publish(msg)


def main():
    rclpy.init()
    node = SimLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
