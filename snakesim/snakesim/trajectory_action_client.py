import os
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from snakesim_interfaces.action import MoveTo

from geometry_msgs.msg import Point

from action_msgs.msg import GoalStatus

from time import sleep
from datetime import datetime

import pandas as pd
import numpy as np


RAD_90 = np.deg2rad(90)


class TrajectoryActionClient(Node):

    def __init__(self, metric_name):
        super().__init__("trajectory_action_client")
        self._action_client = ActionClient(self, MoveTo, "move_to")
        self.status = GoalStatus.STATUS_EXECUTING

        self.metric_data = []
        self.error_data = []
        self.joint_data = []
        self.metric_name = metric_name

    def send_goal(self, point, gain, initial_configuration, metric_name):
        self.status = GoalStatus.STATUS_EXECUTING

        goal_msg = MoveTo.Goal()

        goal_msg.gain = gain
        goal_msg.position = point
        goal_msg.initial_configuration = initial_configuration
        goal_msg.metric_name = metric_name

        self.get_logger().info("Waiting for action server...")

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.metric_data.append(result.score)
        self.error_data.append(result.position_error)
        self.joint_data.append(self.joint_data[-1])
        self.get_logger().info("Result: {}".format(result.score))
        self.get_logger().info("Error: {}".format(result.position_error))
        self.status = GoalStatus.STATUS_SUCCEEDED

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.metric_data.append(feedback.score)
        self.error_data.append(feedback.position_error)
        self.joint_data.append(feedback.current_configuration)

    def save_result(self, filename):
        data = {"metric": self.metric_data, "error": self.error_data}

        for i in range(len(self.joint_data[0])):
            data[f"joint_{i}"] = []

        for jd in self.joint_data:
            for i, j in enumerate(jd):
                data[f"joint_{i}"].append(j)

        df = pd.DataFrame(data)
        df.to_csv(filename, index=False)

        self.metric_data = []
        self.error_data = []
        self.joint_data = []


def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryActionClient("joint_distance")

    np.random.seed(42)

    now_timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

    save_dir = f"data/{now_timestamp}"

    os.makedirs(save_dir, exist_ok=True)

    q0 = np.random.uniform(
        -RAD_90 / 1.5, RAD_90 / 1.5, size=5
    )  # -- joint_distance
    # q0 = np.random.uniform(0, RAD_90 / 1.5, size=5)  # -- manipulability

    for gain in [0.0, 250.0, 500.0, 1000.0]:

        goal = {
            "point":  # Point(
            # x=0.1, y=0.1, z=0.2 ),  # -- manipulability
            Point(x=0.0, y=0.0, z=0.3),  # -- joint_distance
            "gain": gain,
            "initial_configuration": q0,
            "metric_name": "manipulability",
        }

        action_client.get_logger().info("Sending goal...")

        action_client.send_goal(**goal)

        while action_client.status != GoalStatus.STATUS_SUCCEEDED:
            rclpy.spin_once(action_client)

        sleep(1)

        action_client.save_result(
            os.path.join(
                save_dir,
                f"gain={gain}metric={action_client.metric_name}.csv",
            )
        )

    action_client.get_logger().info("All done!")
    action_client.destroy_node()


if __name__ == "__main__":
    main()
