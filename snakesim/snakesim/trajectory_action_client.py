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


N_EXP = 1
RAD_120 = np.deg2rad(120)


class TrajectoryActionClient(Node):

    def __init__(self):
        super().__init__("trajectory_action_client")
        self._action_client = ActionClient(self, MoveTo, "move_to")
        self.status = GoalStatus.STATUS_EXECUTING

        self.curr_exp = 0
        self.metric_data = [[] for _ in range(N_EXP)]
        self.error_data = [[] for _ in range(N_EXP)]
        self.metric_name = "joint_distance"

    def send_goal(self, point, gain, initial_configuration):
        self.status = GoalStatus.STATUS_EXECUTING

        goal_msg = MoveTo.Goal()

        goal_msg.gain = gain
        goal_msg.position = point
        goal_msg.initial_configuration = initial_configuration

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
        self.error_data[self.curr_exp].append(result.position_error)
        self.get_logger().info("Result: {}".format(result.score))
        self.get_logger().info("Error: {}".format(result.position_error))
        self.curr_exp += 1
        self.status = GoalStatus.STATUS_SUCCEEDED

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.metric_data[self.curr_exp].append(feedback.score)
        self.error_data[self.curr_exp].append(feedback.position_error)

    def save_result(self, filename):
        max_len = max([len(v) for v in self.metric_data])

        for i, md in enumerate(self.metric_data):
            self.metric_data[i] = md + [None] * (max_len - len(md))

        self.metric_data = np.array(self.metric_data).T

        df = pd.DataFrame(
            self.metric_data,
            columns=[f"exp_{i}" for i in range(self.metric_data.shape[1])],
        )

        df.to_csv(filename, index=False)

        max_len = max([len(v) for v in self.error_data])

        for i, md in enumerate(self.error_data):
            self.error_data[i] = md + [None] * (max_len - len(md))

        self.error_data = np.array(self.error_data).T

        df = pd.DataFrame(
            self.error_data,
            columns=[f"exp_{i}" for i in range(self.error_data.shape[1])],
        )

        df.to_csv(filename.replace(".csv", "_error.csv"), index=False)

        self.curr_exp = 0
        self.metric_data = [[] for _ in range(N_EXP)]
        self.error_data = [[] for _ in range(N_EXP)]


def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryActionClient()

    # q0 = np.random.uniform(-RAD_120, RAD_120, size=5)

    q0s = np.random.uniform(-RAD_120 / 6.0, RAD_120 / 6.0, size=(N_EXP, 5))
    points = np.random.uniform(-0.1, 0.1, size=N_EXP)
    points = [Point(x=0.1, y=0.1, z=0.1) for point in points]

    # for i in range(N_EXP):
    #    q0s[i, :] = q0s[i, :] + q0

    gains = [0.0, 50.0, 100.0, 500.0]

    now_timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")

    save_dir = f"data/{now_timestamp}"

    os.makedirs(save_dir, exist_ok=True)

    for gain in gains:
        goals = [
            {
                "point": points[i],
                "gain": gain,
                "initial_configuration": q0,
            }
            for i, q0 in enumerate(q0s)
        ]

        for i, goal in enumerate(goals):

            action_client.get_logger().info(
                f"Sending goal {i+1} of {len(goals)}..."
            )

            action_client.send_goal(**goal)

            while action_client.status != GoalStatus.STATUS_SUCCEEDED:
                rclpy.spin_once(action_client)

            sleep(1)

        action_client.save_result(
            os.path.join(save_dir, f"gain={gain}.csv")
        )

    with open(os.path.join(save_dir, "params.txt"), "a") as f:
        f.write(f"metric={action_client.metric_name}\n")
        f.write(f"n_exp={N_EXP}\n")
        f.write(f"n_iter={len(action_client.metric_data[0])}\n")
        f.write(f"point={goals[0]['point']}\n")
        # f.write(f"initial_configuration={q0}\n")

    action_client.get_logger().info("All done!")
    action_client.destroy_node()


if __name__ == "__main__":
    main()
