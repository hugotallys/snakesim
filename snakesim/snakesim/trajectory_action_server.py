from time import sleep

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from snakesim_interfaces.action import MoveTo
from snakesim_interfaces.srv import JointState

from geometry_msgs.msg import Twist, Pose

from std_msgs.msg import Float64


class RRCActionServer(Node):

    def __init__(self, max_iter=5000, tol=0.01):
        super().__init__("go_to_point_action_server")

        self._action_server = ActionServer(
            self, MoveTo, "move_to", self.execute_callback
        )

        self.max_iter = max_iter
        self.tol = tol

        self.get_logger().info("RRC Action Server has been started.")

        service_client_cb_group = MutuallyExclusiveCallbackGroup()

        self.cli = self.create_client(
            JointState,
            "init_joint_state",
            callback_group=service_client_cb_group,
        )

        self.end_effector_pose_subscriber = self.create_subscription(
            Pose,
            "end_effector_pose",
            self.end_effector_pose_callback,
            10,
            callback_group=service_client_cb_group,
        )

        self.metric_subscriber = self.create_subscription(
            Float64,
            "metric",
            self.metric_callback,
            10,
            callback_group=service_client_cb_group,
        )

        self.twist_publisher = self.create_publisher(
            Twist, "target_twist", 10
        )

        self.gain_publisher = self.create_publisher(
            Float64, "target_gain", 10
        )

        self.end_effector_pose = Pose()

    def send_request(self, joint_state):
        request = JointState.Request()
        request.joint_state = joint_state
        response = self.cli.call(request)
        return response

    def end_effector_pose_callback(self, msg):
        self.end_effector_pose = msg

    def metric_callback(self, msg):
        self.metric = msg.data

    def execute_callback(self, goal_handle):
        self.get_logger().info("*** Executing goal ... ***")

        self.send_request(
            joint_state=goal_handle.request.initial_configuration,
        )

        sleep(1)

        feedback_msg = MoveTo.Feedback()

        target_position = self.point_to_array(goal_handle.request.position)

        self.get_logger().info(f"Target position: {target_position}")

        self.gain_publisher.publish(Float64(data=goal_handle.request.gain))

        for _ in range(self.max_iter):
            curr_position = self.end_effector_pose.position

            curr_position_arr = self.point_to_array(
                self.end_effector_pose.position
            )

            alpha = 0.05
            ee_vel = target_position - curr_position_arr
            ee_vel = alpha * ee_vel / np.linalg.norm(ee_vel)

            ee_twist = Twist()

            ee_twist.linear.x = ee_vel[0]
            ee_twist.linear.y = ee_vel[1]
            ee_twist.linear.z = ee_vel[2]

            self.twist_publisher.publish(ee_twist)

            feedback_msg.current_position = curr_position
            feedback_msg.score = self.metric
            feedback_msg.current_configuration = [
                0.0
                for _ in range(
                    len(goal_handle.request.initial_configuration)
                )
            ]

            goal_handle.publish_feedback(feedback_msg)

            curr_position = self.point_to_array(curr_position)

            dist = self.norm(curr_position, target_position)

            feedback_msg.position_error = dist

            if dist < self.tol:
                break

            sleep(0.01)

        self.twist_publisher.publish(Twist())
        self.gain_publisher.publish(Float64(data=0.0))

        goal_handle.succeed()

        result = MoveTo.Result()

        result.score = self.metric
        result.position_error = dist

        self.get_logger().info("*** Goal execution completed ***")

        return result

    @staticmethod
    def point_to_array(point):
        return np.array([point.x, point.y, point.z])

    @staticmethod
    def norm(a, b):
        return np.linalg.norm(a - b)


def main(args=None):
    rclpy.init()
    node = RRCActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info("Beginning client, shut down with CTRL-C")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
