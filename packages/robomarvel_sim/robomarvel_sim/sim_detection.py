"""sim_detection - виртуальная детекция объектов из файла мира.

Объекты берутся из того же world.yaml, что и препятствия, проецируются
в виртуальную камеру и публикуются в том же виде, в каком их даёт настоящий
детектор.

  /detections                          vision_msgs/Detection2DArray
  /camera/camera_info                  sensor_msgs/CameraInfo
  /camera/image_annotated_compressed   sensor_msgs/CompressedImage
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from robomarvel_sim.world import World

try:
    import cv2
except ImportError:  # отрисовка необязательна
    cv2 = None


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimDetection(Node):
    def __init__(self):
        super().__init__("sim_detection")

        self.declare_parameter("world", "")
        self.declare_parameter("rate", 5.0)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("hfov_deg", 70.0)
        self.declare_parameter("max_range", 6.0)
        self.declare_parameter("optical_frame", "camera_optical_frame")
        # положение камеры относительно base_link (должно совпадать с robot.urdf)
        self.declare_parameter("camera_x", 0.08)
        self.declare_parameter("camera_z", 0.06)
        self.declare_parameter("publish_image", True)

        world_path = self.get_parameter("world").value
        if not world_path:
            raise RuntimeError("Параметр 'world' обязателен для sim_detection")
        self.world = World.from_yaml(world_path)

        self.rate = float(self.get_parameter("rate").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.hfov = math.radians(float(self.get_parameter("hfov_deg").value))
        self.max_range = float(self.get_parameter("max_range").value)
        self.optical_frame = self.get_parameter("optical_frame").value
        self.cam_x = float(self.get_parameter("camera_x").value)
        self.cam_z = float(self.get_parameter("camera_z").value)
        self.publish_image = bool(self.get_parameter("publish_image").value) and cv2 is not None

        self.fx = (self.w / 2.0) / math.tan(self.hfov / 2.0)
        self.fy = self.fx
        self.cx = self.w / 2.0
        self.cy = self.h / 2.0

        self.pose = None

        self.det_pub = self.create_publisher(Detection2DArray, "/detections", 5)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 5)
        self.img_pub = self.create_publisher(
            CompressedImage, "/camera/image_annotated_compressed", 2
        )
        self.create_subscription(Odometry, "/sim/ground_truth", self.truth_callback, 5)
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

        self.get_logger().info(
            f"sim_detection: {len(self.world.objects)} объектов, "
            f"{self.w}x{self.h}, hFoV={math.degrees(self.hfov):.0f} град, fx={self.fx:.1f}"
        )

    def truth_callback(self, msg: Odometry):
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            quat_to_yaw(msg.pose.pose.orientation),
        )

    def camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.optical_frame
        info.width = self.w
        info.height = self.h
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fy, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def tick(self):
        if self.pose is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.info_pub.publish(self.camera_info(stamp))

        bx, by, byaw = self.pose
        cos_b, sin_b = math.cos(byaw), math.sin(byaw)
        # оптический центр камеры в мировых координатах
        ox = bx + cos_b * self.cam_x
        oy = by + sin_b * self.cam_x

        arr = Detection2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self.optical_frame

        boxes = []
        for idx, obj in enumerate(self.world.objects):
            dx = obj.x - ox
            dy = obj.y - oy
            dist = math.hypot(dx, dy)
            if dist > self.max_range or dist < 1e-3:
                continue

            # в систему координат камеры: forward - вдоль оси робота, left - влево
            forward = cos_b * dx + sin_b * dy
            left = -sin_b * dx + cos_b * dy
            if forward <= 0.05:
                continue

            bearing = math.atan2(left, forward)
            if abs(bearing) > self.hfov / 2.0:
                continue

            # перекрыт ли объект стеной или другим объектом
            world_angle = math.atan2(dy, dx)
            free = self.world.range_to(ox, oy, world_angle)
            if free < dist - obj.r - 0.05:
                continue

            # оптическая система: X вправо, Y вниз, Z вперёд
            xc = -left
            zc = forward
            yc = self.cam_z - (obj.z + obj.height / 2.0)

            u = self.fx * xc / zc + self.cx
            v = self.fy * yc / zc + self.cy
            box_w = 2.0 * self.fx * obj.r / zc
            box_h = self.fy * obj.height / zc

            if u < -box_w or u > self.w + box_w:
                continue

            det = Detection2D()
            det.header = arr.header
            det.id = f"{obj.cls}_{idx}"
            bb = BoundingBox2D()
            bb.center.position.x = float(u)
            bb.center.position.y = float(v)
            bb.center.theta = 0.0
            bb.size_x = float(box_w)
            bb.size_y = float(box_h)
            det.bbox = bb

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = obj.cls
            hyp.hypothesis.score = 0.95
            hyp.pose.pose.position = Point(x=float(xc), y=float(yc), z=float(zc))
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)

            arr.detections.append(det)
            boxes.append((obj.cls, u, v, box_w, box_h, dist))

        self.det_pub.publish(arr)
        if self.publish_image:
            self.draw(boxes, stamp)

    def draw(self, boxes, stamp):
        img = np.full((self.h, self.w, 3), 40, dtype=np.uint8)
        cv2.putText(img, "SIM (virtual detections)", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2)
        for cls, u, v, bw, bh, dist in boxes:
            x1, y1 = int(u - bw / 2), int(v - bh / 2)
            x2, y2 = int(u + bw / 2), int(v + bh / 2)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 220, 0), 2)
            cv2.putText(img, f"{cls} {dist:.2f}m", (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.optical_frame
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self.img_pub.publish(msg)


def main():
    rclpy.init()
    node = SimDetection()
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
