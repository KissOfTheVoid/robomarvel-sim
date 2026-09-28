import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from threading import Thread
import time
import cv2


class RtspImagePublisher(Node):
    def __init__(self):
        super().__init__("rtsp_image_publisher")

        self.declare_parameter("rtsp_url", "rtsp://localhost:8554/cam")
        self.declare_parameter("topic", "/camera/image_raw")
        self.declare_parameter("frame_id", "/camera/image_raw")
        self.declare_parameter("resolution", "640x480")
        self.declare_parameter("fps", 10.0)
        self.declare_parameter("use_compression", False)

        self.rtsp_url = self.get_parameter("rtsp_url").get_parameter_value().string_value
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.resolution = self.get_parameter("resolution").get_parameter_value().string_value
        self.fps = self.get_parameter("fps").get_parameter_value().double_value
        self.use_compression = self.get_parameter("use_compression").get_parameter_value().bool_value

        self.cv_bridge = CvBridge()
        msg_type = Image if not self.use_compression else CompressedImage
        self.publisher = self.create_publisher(msg_type, self.topic, 1)
        self.cap = None
        self.last_frame = None

        self.read_thread = Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Streaming from {self.rtsp_url} to topic '{self.topic}' [{self.resolution} @ {self.fps} FPS]")

    def init_capture(self):
        while True:
            width, height = self.resolution.split("x")
            gst_str = (
                f"rtspsrc location={self.rtsp_url} protocols=tcp latency=0 ! rtph264depay ! decodebin ! "
                f"videoscale add-borders=true ! video/x-raw,width={width},height={height},pixel-aspect-ratio=1/1 ! "
                "videoconvert ! appsink drop=true sync=false"
            )
            cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                self.get_logger().info("Video capture ready")
                return cap
            self.get_logger().error("Failed to init video capture, retrying...")
            time.sleep(5)

    def read_loop(self):
        while rclpy.ok():
            if self.cap is None:
                self.cap = self.init_capture()
            else:
                ret, frame = self.cap.read()
                if ret:
                    self.last_frame = frame
                else:
                    self.get_logger().error("Video capture closed, reconnecting...")
                    self.last_frame = None
                    self.cap = None
        if self.cap is not None:
            self.cap.release()

    def timer_callback(self):
        if self.last_frame is None: return
        if self.use_compression:
            msg = self.cv_bridge.cv2_to_compressed_imgmsg(self.last_frame, dst_format="jpeg")
        else:
            msg = self.cv_bridge.cv2_to_imgmsg(self.last_frame, encoding="bgr8")

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RtspImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
