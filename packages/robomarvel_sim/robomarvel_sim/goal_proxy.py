"""goal_proxy - мост между топиком /goal и action-сервером Nav2.

Та же нода, что работает на роботе.

  /goal           (подписка)  PoseStamped        - цель навигации
  /goal/status    (издание)   String             - GOAL ACCEPTED / SUCCESS / CANCELLED / ABORTED
  /goal/feedback  (издание)   Float32MultiArray  - [distance_remaining, nav_time, recovery_count]
  /goal/cancel    (сервис)    std_srvs/Trigger   - отмена текущей цели
"""

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

STATUS_TEXT = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCESS",
    GoalStatus.STATUS_CANCELED: "CANCELLED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class GoalProxy(Node):
    def __init__(self):
        super().__init__("goal_proxy")

        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None

        self.create_subscription(PoseStamped, "/goal", self.goal_callback, 1)
        self.status_pub = self.create_publisher(String, "/goal/status", 10)
        self.feedback_pub = self.create_publisher(Float32MultiArray, "/goal/feedback", 10)
        self.create_service(Trigger, "/goal/cancel", self.cancel_service)

        self.get_logger().info("goal_proxy запущен, жду цели в /goal")

    def say(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def goal_callback(self, msg: PoseStamped):
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.say("NAV2 UNAVAILABLE")
            return

        # новая цель во время активной навигации отменяет текущую
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose = msg
        if not goal.pose.header.frame_id:
            goal.pose.header.frame_id = "map"

        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.say("GOAL REJECTED")
            return
        self.goal_handle = handle
        self.say("GOAL ACCEPTED")
        handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, feedback):
        fb = feedback.feedback
        msg = Float32MultiArray()
        msg.data = [
            float(fb.distance_remaining),
            float(fb.navigation_time.sec + fb.navigation_time.nanosec * 1e-9),
            float(fb.number_of_recoveries),
        ]
        self.feedback_pub.publish(msg)

    def on_result(self, future):
        status = future.result().status
        self.goal_handle = None
        self.say(STATUS_TEXT.get(status, f"UNKNOWN({status})"))

    def cancel_service(self, request, response):
        if self.goal_handle is None:
            response.success = False
            response.message = "Нет активной цели"
            return response
        self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        response.success = True
        response.message = "Цель отменена"
        self.say("CANCEL REQUESTED")
        return response


def main():
    rclpy.init()
    node = GoalProxy()
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
