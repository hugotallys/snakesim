import os
from datetime import datetime
from time import sleep

import numpy as np
import pandas as pd
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from snakesim_interfaces.action import TrajectoryRRC


class TrajectoryActionClient(Node):

    def __init__(self, metric_name):
        super().__init__("trajectory_action_client")
        self._action_client = ActionClient(
            self, TrajectoryRRC, "trajectory_rrc"
        )
        self.status = GoalStatus.STATUS_EXECUTING

        self.metric_data = []
        self.position_data = []
        self.desired_position_data = []
        self.joint_data = []
        self.metric_name = metric_name

    def send_goal(
        self,
        gain,
        initial_configuration,
        target_configuration,
        metric_name,
    ):
        self.status = GoalStatus.STATUS_EXECUTING

        goal_msg = TrajectoryRRC.Goal()

        goal_msg.gain = gain
        goal_msg.initial_configuration = initial_configuration
        goal_msg.target_configuration = target_configuration

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
            self.get_logger().info("Goal rejected 😔")
        else:
            self.get_logger().info("Goal accepted 😊")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.metric_data.append(result.score)
        self.position_data.append(result.current_position)
        self.desired_position_data.append(result.desired_position)
        self.joint_data.append(result.current_configuration)
        self.status = GoalStatus.STATUS_SUCCEEDED

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.metric_data.append(feedback.score)
        self.position_data.append(feedback.current_position)
        self.desired_position_data.append(feedback.desired_position)
        self.joint_data.append(feedback.current_configuration)

    def save_result(self, filename):
        data = {"metric": self.metric_data}

        for i in range(len(self.joint_data[0])):
            data[f"joint_{i}"] = []

        for jd in self.joint_data:
            for i, j in enumerate(jd):
                data[f"joint_{i}"].append(j)

        for i, value in enumerate(["x", "y", "z"]):
            data[value] = []

        for pos in self.position_data:
            data["x"].append(pos.x)
            data["y"].append(pos.y)
            data["z"].append(pos.z)

        for i, value in enumerate(["x_", "y_", "z_"]):
            data[value] = []

        for pos in self.desired_position_data:
            data["x_"].append(pos.x)
            data["y_"].append(pos.y)
            data["z_"].append(pos.z)

        df = pd.DataFrame(data)
        df.to_csv(filename, index=False)

        self.metric_data = []
        self.position_data = []
        self.desired_position_data = []
        self.joint_data = []


def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryActionClient("joint_distance")

    # np.random.seed(42)

    now_timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

    save_dir = f"data/{now_timestamp}"

    os.makedirs(save_dir, exist_ok=True)

    max_angle = np.deg2rad(90)

    q0 = np.random.uniform(-max_angle, max_angle, size=5)
    qf = np.random.uniform(-max_angle, max_angle, size=5)

    for gain in [0.0, 100.0, 200.0, 400.0]:

        goal = {
            "gain": gain,
            "initial_configuration": q0,
            "target_configuration": qf,
            "metric_name": action_client.metric_name,
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
