import rclpy
import numpy as np

from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock
from snakesim_interfaces.action import MoveTo
from geometry_msgs.msg import Pose

from rclpy.executors import MultiThreadedExecutor

from roboticstoolbox import DHRobot, RevoluteDH
from time import sleep


class Robot:
    def __init__(self):

        self.qlim = [-np.pi, np.pi]

        self.robot = DHRobot(
            [
                RevoluteDH(d=0, a=0.06, alpha=np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=-np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=-np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=-np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.026, alpha=np.pi / 2, qlim=self.qlim),
            ],
            name="Snake",
        )

    def jacobian(self, q):
        return self.robot.jacob0(q, half="trans")[:3, :]

    def q0dot(self, q, k0=0.0):
        n = len(q)
        values = []
        qlims = self.robot.qlim.T
        for i in range(qlims.shape[0]):
            q_min, q_max = qlims[i][0], qlims[i][1]
            q_mean = 0.5 * (q_max + q_min)
            values.append((q[i] - q_mean) / (q_max - q_min) ** 2)
        return (-k0 / n) * (np.array(values)).reshape(n, 1)


class PubSub(Node):

    def __init__(self):
        super().__init__("pub_sub")

        self.n_joints = 7

        self.q = np.zeros(self.n_joints)
        self.q_ = self.q.copy()

        self.k0 = None

        self.dx = np.zeros(3)

        self.ee_pose = Pose()

        self.robot = Robot()

        self.curr_time = 0.0
        self.start_time = None

    def init_pub_subs(self):
        self.publisher_ = self.create_publisher(
            Float64MultiArray, "targetPosition", 10
        )

        self.subscriber_ = self.create_subscription(
            Float64MultiArray, "position", self.position_callback, 10
        )

        self.clock_subscriber_ = self.create_subscription(
            Clock, "clock", self.simtime_callback, 10
        )

        self.subscriber_ee = self.create_subscription(
            Pose, "endEffPose", self.pose_callback, 10
        )

    def set_joint_position(self):
        msg = Float64MultiArray(data=self.q.tolist())
        self.publisher_.publish(msg)

    def position_callback(self, msg):
        self.q_ = np.array(msg.data)

    def pose_callback(self, msg):
        self.ee_pose = msg

    def simtime_callback(self, msg):
        simulation_time = msg.clock.sec + msg.clock.nanosec / 1e9

        if self.start_time is None:
            self.start_time = simulation_time

        if simulation_time - self.start_time > 5.0:
            dt = simulation_time - self.curr_time

            J = self.robot.jacobian(self.q_)
            JT = np.linalg.pinv(J)

            q0dot = self.robot.q0dot(self.q_, k0=self.k0)

            dq = (JT @ self.dx).reshape(-1, 1) + (
                np.eye(self.n_joints) - JT @ J
            ) @ q0dot

            self.q = np.clip(self.q_ + dq.flatten() * dt, -3.14, 3.14)

        self.set_joint_position()
        self.curr_time = simulation_time


class RRCActionServer(Node):

    def __init__(self, pubsub):
        super().__init__("rrc_action_server")

        self._action_server = ActionServer(
            self, MoveTo, "move_to", self.execute_callback
        )

        self.pubsub = pubsub

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        self.pubsub.k0 = goal_handle.request.gain

        self.pubsub.q = np.array(goal_handle.request.initial_configuration)

        self.pubsub.init_pub_subs()

        self.pubsub.set_joint_position()

        feedback_msg = MoveTo.Feedback()

        target_position = np.array(
            [
                goal_handle.request.position.x,
                goal_handle.request.position.y,
                goal_handle.request.position.z,
            ]
        )

        curr_target_position = np.array(
            [
                self.pubsub.ee_pose.position.x,
                self.pubsub.ee_pose.position.y,
                self.pubsub.ee_pose.position.z,
            ]
        )

        alpha = 0.1
        self.pubsub.dx = target_position - curr_target_position
        self.pubsub.dx = alpha * self.pubsub.dx

        while True:
            feedback_msg.current_position = self.pubsub.ee_pose.position
            feedback_msg.partial_score = 0.0
            feedback_msg.current_configuration = self.pubsub.q_.tolist()

            goal_handle.publish_feedback(feedback_msg)

            curr_target_position = np.array(
                [
                    self.pubsub.ee_pose.position.x,
                    self.pubsub.ee_pose.position.y,
                    self.pubsub.ee_pose.position.z,
                ]
            )

            if (
                np.linalg.norm(target_position - curr_target_position)
                < 0.05
            ):
                break

            sleep(0.032)

        goal_handle.succeed()

        result = MoveTo.Result()

        result.score = 0.0
        result.position_error = 0.0

        return result


def main(args=None):
    rclpy.init(args=args)
    try:
        py_pub_sub = PubSub()
        py_action_server = RRCActionServer(pubsub=py_pub_sub)

        executor = MultiThreadedExecutor(num_threads=4)

        executor.add_node(py_action_server)
        executor.add_node(py_pub_sub)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            py_action_server.destroy_node()
            py_pub_sub.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
