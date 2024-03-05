from time import sleep

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from snakesim_interfaces.action import MoveTo

from .resolved_rate_controller import ResolvedRateController


class RRCActionServer(Node):

    def __init__(self, controller, max_iter=1000, tol=0.1):
        super().__init__("rrc_action_server")

        self._action_server = ActionServer(
            self, MoveTo, "move_to", self.execute_callback
        )

        self.controller = controller

        self.max_iter = max_iter
        self.tol = tol

    def execute_callback(self, goal_handle):
        self.get_logger().info("*** Executing goal ... ***")

        self.controller.set_controller_params(
            k0=goal_handle.request.gain,
            q=np.array(goal_handle.request.initial_configuration),
        )

        self.controller.init_pub_subs()

        self.controller.set_joint_position()

        sleep(1)

        feedback_msg = MoveTo.Feedback()

        target_position = self.point_to_array(goal_handle.request.position)

        curr_position = self.point_to_array(
            self.controller.get_ee_position()
        )

        alpha = 0.5
        ee_vel = alpha * (target_position - curr_position)
        self.controller.set_ee_vel(ee_vel)

        for _ in range(self.max_iter):
            curr_position = self.controller.get_ee_position()

            feedback_msg.current_position = curr_position
            feedback_msg.partial_score = 0.0
            feedback_msg.current_configuration = (
                self.controller.get_joint_position()
            )

            goal_handle.publish_feedback(feedback_msg)

            curr_position = self.point_to_array(curr_position)

            dist = self.norm(curr_position, target_position)

            if dist < self.tol:
                self.controller.set_ee_vel(np.zeros(3))
                break

            sleep(0.01)

        goal_handle.succeed()

        result = MoveTo.Result()

        result.score = 0.0
        result.position_error = dist

        return result

    @staticmethod
    def point_to_array(point):
        return np.array([point.x, point.y, point.z])

    @staticmethod
    def norm(a, b):
        return np.linalg.norm(a - b)


def main(args=None):
    rclpy.init(args=args)
    try:
        controller_node = ResolvedRateController()
        action_node = RRCActionServer(
            controller=controller_node, max_iter=5000, tol=0.1
        )

        executor = MultiThreadedExecutor(num_threads=4)

        executor.add_node(action_node)
        executor.add_node(controller_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            action_node.destroy_node()
            controller_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
