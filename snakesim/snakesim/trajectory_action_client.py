import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from snakesim_interfaces.action import MoveTo

from geometry_msgs.msg import Point

from action_msgs.msg import GoalStatus

from time import sleep


class TrajectoryActionClient(Node):

    def __init__(self):
        super().__init__("trajectory_action_client")
        self._action_client = ActionClient(self, MoveTo, "move_to")
        self.status = GoalStatus.STATUS_EXECUTING

    def send_goal(self, point, initial_configuration):
        self.status = GoalStatus.STATUS_EXECUTING

        goal_msg = MoveTo.Goal()

        goal_msg.gain = 0.0
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
        self.get_logger().info("Result: {}".format(result.score))
        self.status = GoalStatus.STATUS_SUCCEEDED

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            "Received feedback: {0}".format(feedback.partial_score)
        )


def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryActionClient()

    inital_configurations = [
        [
            0.5,
            0.5,
            -0.5,
            -0.5,
            1.0,
            -1.0,
            0.0,
        ],
        [
            -0.5,
            -0.5,
            0.5,
            0.5,
            -1.0,
            1.0,
            0.0,
        ],
        [
            1.5,
            0.5,
            0.5,
            0.5,
            -1.0,
            -1.0,
            1.5,
        ],
    ]

    goals = [
        Point(x=0.1, y=0.1, z=0.1),
        Point(x=0.2, y=0.2, z=0.2),
        Point(x=0.1, y=0.2, z=0.1),
    ]

    for init_c, goal in zip(inital_configurations, goals):

        action_client.send_goal(goal, init_c)

        while action_client.status != GoalStatus.STATUS_SUCCEEDED:
            rclpy.spin_once(action_client)

        sleep(1)

    action_client.get_logger().info("All done!")
    action_client.destroy_node()


if __name__ == "__main__":
    main()
