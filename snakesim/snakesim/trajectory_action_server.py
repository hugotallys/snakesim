from time import sleep

import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from snakesim_interfaces.action import TrajectoryRRC
from snakesim_interfaces.srv import SetJointState
from snakesim_interfaces.msg import InputRRC, OutputRRC

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import JointState

from .robot_controller import Robot


class RRCActionServer(Node):

    def __init__(self, max_iter=5000, tol=0.01):
        super().__init__("trajectory_rrc_action_server")

        self._action_server = ActionServer(
            self,
            TrajectoryRRC,
            "trajectory_rrc",
            self.execute_callback,
        )

        self._action_server.register_goal_callback(self.goal_callback)

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

        self.joint_states_subscriber = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_states_callback,
            10,
        )
        self.joint_states_position = None

        self.end_effector = Point()

        self.robot = Robot()

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

    def joint_states_callback(self, msg):
        self.joint_states_position = msg.position

    def goal_callback(self, goal_request):
        min_height = 0.05
        initial_position = self.robot.get_fkine_position(
            goal_request.initial_configuration
        )
        target_position = self.robot.get_fkine_position(
            goal_request.target_configuration
        )

        if (
            initial_position[2] < min_height
            or target_position[2] < min_height
        ):
            self.get_logger().info(
                "*** Invalid goal request ***"
                "\nEnd-effector might collide with floor."
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("*** Executing Goal. ***")

        self.send_request(
            joint_position=goal_handle.request.initial_configuration,
        )

        sleep(1)

        feedback_msg = TrajectoryRRC.Feedback()

        target_position = self.robot.get_fkine_position(
            goal_handle.request.target_configuration
        )

        for _ in range(self.max_iter):
            curr_position = self.end_effector

            curr_position_arr = self.point_to_array(self.end_effector)

            ee_vel = target_position - curr_position_arr

            ee_twist = Twist()

            ee_twist.linear.x = ee_vel[0]
            ee_twist.linear.y = ee_vel[1]
            ee_twist.linear.z = ee_vel[2]

            msg = InputRRC()

            msg.twist = ee_twist
            msg.gain = goal_handle.request.gain
            msg.metric_name = goal_handle.request.metric_name

            self.rrc_input_publisher.publish(msg)

            dist = self.norm(curr_position_arr, target_position)

            feedback_msg.score = self.score
            feedback_msg.current_position = self.end_effector
            feedback_msg.desired_position = Point(
                x=curr_position_arr[0] + ee_vel[0],
                y=curr_position_arr[1] + ee_vel[1],
                z=curr_position_arr[2] + ee_vel[2],
            )

            if self.joint_states_position is not None:
                feedback_msg.current_configuration = (
                    self.joint_states_position
                )
            else:
                feedback_msg.current_configuration = (
                    goal_handle.request.initial_configuration
                )

            goal_handle.publish_feedback(feedback_msg)

            if dist < self.tol:
                break

            sleep(0.032)

        self.rrc_input_publisher.publish(
            InputRRC(
                twist=Twist(),
                gain=0.0,
                metric_name=goal_handle.request.metric_name,
            )
        )

        goal_handle.succeed()

        result = TrajectoryRRC.Result()

        result.score = self.score
        result.current_position = curr_position
        result.desired_position = Point(
            x=curr_position_arr[0] + ee_vel[0],
            y=curr_position_arr[1] + ee_vel[1],
            z=curr_position_arr[2] + ee_vel[2],
        )
        result.current_configuration = self.joint_states_position

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
