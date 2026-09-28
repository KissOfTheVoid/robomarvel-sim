"""Драйвер робота для симулятора: принимает команды, двигает робота, докладывает о себе.

Публикует ровно тот же набор топиков, с теми же типами сообщений и с той же
неполнотой данных, что и драйвер настоящего робота:

  /cmd_vel            подписка, команды движения
  /hardware/status    Float32MultiArray [v_left, left_speed, left_angle,
                                         v_right, right_speed, right_angle]
  /hardware/odom      Odometry, заполнено только twist.twist.linear.x
  /hardware/imu       Imu, заполнена только angular_velocity
  /hardware/battery   Float32, напряжение батареи

Только для симуляции, на роботе этого нет:
  /sim/ground_truth   Odometry с истинной позой, системы координат sim_world и sim_base

Системы координат sim_world и sim_base намеренно названы не так, как принятые
у робота, чтобы ни одна нода не могла случайно опереться на истинную позу.
"""

import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float32MultiArray
from tf2_ros import TransformBroadcaster

from robomarvel_sim import kinematics as kin
from robomarvel_sim.world import World, yaw_to_quat

TWO_PI = 2.0 * math.pi


class SimHardwareNode(Node):
    def __init__(self):
        super().__init__("sim_hwnode")

        self.declare_parameter("world", "")
        self.declare_parameter("telemetry_hz", 50.0)
        self.declare_parameter("publish_odom_tf", False)
        self.declare_parameter("wheel_noise", 0.0)  # разброс проскальзывания колеса
        self.declare_parameter("encoder_noise", 0.02)  # рад/с, шум измерения скорости колеса
        self.declare_parameter("gyro_noise", 0.004)  # рад/с
        self.declare_parameter("gyro_bias", 0.002)  # рад/с, постоянный уход нуля
        self.declare_parameter("battery_capacity_ah", 5.0)
        self.declare_parameter("battery_speedup", 1.0)
        self.declare_parameter("collisions", True)
        self.declare_parameter("robot_radius", 0.08)

        world_path = self.get_parameter("world").value
        self.world = None
        if world_path:
            self.world = World.from_yaml(world_path)
            self.x, self.y, self.yaw = self.world.start
            self.get_logger().info(
                f"Стартовая поза из мира '{self.world.name}': "
                f"x={self.x:.2f} y={self.y:.2f} yaw={self.yaw:.2f}"
            )
        else:
            self.x = self.y = self.yaw = 0.0

        self.collisions = bool(self.get_parameter("collisions").value) and self.world is not None
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        # 16 лучей достаточно, чтобы поймать касание корпусом
        self._probe = np.arange(16) / 16.0 * 2.0 * math.pi
        self.blocked = False

        self.telemetry_hz = float(self.get_parameter("telemetry_hz").value)
        self.publish_odom_tf = bool(self.get_parameter("publish_odom_tf").value)
        self.wheel_noise = float(self.get_parameter("wheel_noise").value)
        self.encoder_noise = float(self.get_parameter("encoder_noise").value)
        self.gyro_noise = float(self.get_parameter("gyro_noise").value)
        self.gyro_bias = float(self.get_parameter("gyro_bias").value)
        self.capacity_ah = float(self.get_parameter("battery_capacity_ah").value)
        self.battery_speedup = float(self.get_parameter("battery_speedup").value)

        # состояние контура управления
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        self.v_left = 0.0
        self.v_right = 0.0

        # истинное состояние, известное только симулятору
        self.true_v = 0.0
        self.true_w = 0.0
        self.left_angle = 0.0
        self.right_angle = 0.0

        self.soc = 1.0
        self.battery_v = None

        self.last_cmd_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 1)
        self.imu_pub = self.create_publisher(Imu, "/hardware/imu", 1)
        self.status_pub = self.create_publisher(Float32MultiArray, "/hardware/status", 1)
        self.odom_pub = self.create_publisher(Odometry, "/hardware/odom", 1)
        self.battery_pub = self.create_publisher(Float32, "/hardware/battery", 1)
        self.truth_pub = self.create_publisher(Odometry, "/sim/ground_truth", 1)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.control_timer = self.create_timer(1.0 / kin.CONTROL_HZ, self.update)
        self.telemetry_timer = self.create_timer(1.0 / self.telemetry_hz, self.telemetry)

        if self.publish_odom_tf:
            self.get_logger().warn(
                "publish_odom_tf=true: публикую odom->base_link из истинной позы. "
                "Это отладочный режим."
            )
        self.get_logger().info("sim_hwnode запущен")

    def cmd_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.target_v, self.target_w = kin.shape_command(msg.linear.x, msg.angular.z)

    def update(self):
        dt = 1.0 / kin.CONTROL_HZ

        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > kin.CMD_TIMEOUT:
            self.target_v = 0.0
            self.target_w = 0.0

        self.current_v, self.current_w = kin.ramp_body(
            self.current_v, self.current_w, self.target_v, self.target_w
        )
        self.v_left, self.v_right = kin.body_to_wheels(self.current_v, self.current_w)

        # колёса крутятся с заданной скоростью, плюс необязательное проскальзывание
        wl = self.v_left
        wr = self.v_right
        if self.wheel_noise > 0.0:
            wl *= 1.0 + random.gauss(0.0, self.wheel_noise)
            wr *= 1.0 + random.gauss(0.0, self.wheel_noise)

        self.true_v, self.true_w = kin.wheels_to_body(wl, wr)
        nx, ny, nyaw = kin.integrate_pose(
            self.x, self.y, self.yaw, self.true_v, self.true_w, dt
        )

        if self.collisions and self._hits_obstacle(nx, ny):
            # Упёрлись: корпус не двигаем, но поворот разрешаем - иначе робот
            # останется у стены навсегда и вывести его оттуда будет нечем.
            self.yaw = nyaw
            self.true_v = 0.0
            if not self.blocked:
                self.get_logger().warn(
                    "Столкновение со стеной: корпус заблокирован. "
                    "Развернитесь на месте (j / l) и отъезжайте."
                )
                self.blocked = True
        else:
            self.x, self.y, self.yaw = nx, ny, nyaw
            if self.blocked:
                self.get_logger().info("Препятствие обойдено, движение восстановлено")
                self.blocked = False

        self.left_angle = (self.left_angle + wl * dt) % TWO_PI
        self.right_angle = (self.right_angle + wr * dt) % TWO_PI

        drain_a = 0.5 + 1.5 * min(1.0, abs(self.true_v) / kin.MAX_LINEAR)
        self.soc = max(
            0.0, self.soc - drain_a * dt * self.battery_speedup / 3600.0 / self.capacity_ah
        )

        self.publish_truth()

    def _hits_obstacle(self, x, y):
        """Корпус робота считаем окружностью радиуса robot_radius вокруг его центра."""
        return float(self.world.raycast(x, y, self._probe).min()) < self.robot_radius

    def telemetry(self):
        now = self.get_clock().now().to_msg()

        left_speed = self.v_left + random.gauss(0.0, self.encoder_noise)
        right_speed = self.v_right + random.gauss(0.0, self.encoder_noise)

        status = Float32MultiArray()
        status.data = [
            float(self.v_left),
            float(left_speed),
            float(self.left_angle),
            float(self.v_right),
            float(right_speed),
            float(self.right_angle),
        ]
        self.status_pub.publish(status)

        # 8.3 В при полном заряде, 6.0 В при пустом
        voltage = 6.0 + 2.3 * self.soc
        if self.battery_v is None:
            self.battery_v = voltage
        else:
            self.battery_v = 0.999 * self.battery_v + 0.001 * voltage
        batt = Float32()
        batt.data = float(self.battery_v)
        self.battery_pub.publish(batt)

        imu = Imu()
        imu.header.frame_id = "base_link"
        imu.header.stamp = now
        imu.orientation_covariance = [-1.0] + [0.0] * 8
        imu.linear_acceleration_covariance = [-1.0] + [0.0] * 8
        imu.angular_velocity.x = random.gauss(0.0, self.gyro_noise)
        imu.angular_velocity.y = random.gauss(0.0, self.gyro_noise)
        imu.angular_velocity.z = self.true_w + self.gyro_bias + random.gauss(0.0, self.gyro_noise)
        cov = [0.0] * 9
        cov[0] = cov[4] = cov[8] = 0.001
        imu.angular_velocity_covariance = cov
        self.imu_pub.publish(imu)

        odom = Odometry()
        # На роботе оба поля равны "base_link" - повторяем как есть, чтобы
        # симуляция не выглядела правильнее железа.
        odom.header.frame_id = "base_link"
        odom.child_frame_id = "base_link"
        odom.header.stamp = now
        odom.twist.twist.linear.x = (left_speed + right_speed) * kin.WHEEL_RADIUS / 2.0
        twist_cov = [0.0] * 36
        twist_cov[0] = 0.001
        twist_cov[35] = 1000.0
        if abs(left_speed) < 0.001 and abs(right_speed) < 0.001:
            twist_cov[0] = 0.000001
            twist_cov[35] = 0.000001
        odom.twist.covariance = twist_cov
        self.odom_pub.publish(odom)

    def publish_truth(self):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "sim_world"
        odom.child_frame_id = "sim_base"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.true_v
        odom.twist.twist.angular.z = self.true_w
        self.truth_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "sim_world"
        tf.child_frame_id = "sim_base"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        if self.publish_odom_tf:
            tf2 = TransformStamped()
            tf2.header.stamp = now
            tf2.header.frame_id = "odom"
            tf2.child_frame_id = "base_link"
            tf2.transform.translation.x = self.x
            tf2.transform.translation.y = self.y
            tf2.transform.rotation.x = qx
            tf2.transform.rotation.y = qy
            tf2.transform.rotation.z = qz
            tf2.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf2)


def main():
    rclpy.init()
    node = SimHardwareNode()
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
