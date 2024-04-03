from time import sleep

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from snakesim_interfaces.action import MoveTo
from snakesim_interfaces.srv import SetJointState
from snakesim_interfaces.msg import InputRRC, OutputRRC

from geometry_msgs.msg import Twist, Point

from std_msgs.msg import Float64


class RRCActionServer(Node):

    def __init__(self, max_iter=1000, tol=0.01):
        super().__init__("go_to_point_action_server")

        self._action_server = ActionServer(
            self, MoveTo, "move_to", self.execute_callback
        )

        self.max_iter = max_iter
        self.tol = tol

        self.get_logger().info("RRC Action Server has been started.")

        service_client_cb_group = MutuallyExclusiveCallbackGroup()

        self.cli = self.create_client(
            SetJointState,
            "set_joint_state",
            callback_group=service_client_cb_group,
        )

        self.rrc_input_publisher = self.create_publisher(
            InputRRC, "rrc_input", 10
        )

        self.rrc_output_subscriber = self.create_subscription(
            OutputRRC,
            "rrc_output",
            self.rrc_output_callback,
            10,
            callback_group=service_client_cb_group,
        )

        self.end_effector = Point()

    def send_request(self, joint_position):
        request = SetJointState.Request()
        request.joint_states.position = [
            float(joint) for joint in joint_position
        ]
        response = self.cli.call(request)
        return response

    def rrc_output_callback(self, msg):
        self.score = msg.score
        self.end_effector = msg.end_effector

    def execute_callback(self, goal_handle):
        self.get_logger().info("*** Executing goal ... ***")

        self.send_request(
            joint_position=goal_handle.request.initial_configuration,
        )

        sleep(1)

        feedback_msg = MoveTo.Feedback()

        target_position = self.point_to_array(goal_handle.request.position)

        self.get_logger().info(f"Target position: {target_position}")

        for _ in range(self.max_iter):
            curr_position = self.end_effector

            curr_position_arr = self.point_to_array(self.end_effector)

            alpha = 0.1
            ee_vel = target_position - curr_position_arr
            ee_vel = alpha * ee_vel / np.linalg.norm(ee_vel)

            ee_twist = Twist()

            ee_twist.linear.x = ee_vel[0]
            ee_twist.linear.y = ee_vel[1]
            ee_twist.linear.z = ee_vel[2]

            msg = InputRRC()

            msg.twist = ee_twist
            msg.gain = goal_handle.request.gain
            msg.metric_name = (
                "joint_distance"  # goal_handle.request.metric_name
            )

            self.rrc_input_publisher.publish(msg)

            feedback_msg.current_position = curr_position
            feedback_msg.score = self.score
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

            sleep(0.1)

        self.rrc_input_publisher.publish(
            InputRRC(twist=Twist(), gain=0.0, metric_name="joint_distance")
        )

        goal_handle.succeed()

        result = MoveTo.Result()

        result.score = self.score
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
