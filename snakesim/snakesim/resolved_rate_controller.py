import numpy as np
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray

from .robot import Robot


class ResolvedRateController(Node):

    def __init__(self):
        super().__init__("pub_sub")

        self.n_joints = 7

        self.q = np.zeros(self.n_joints)
        self.q_ = self.q.copy()

        self.k0 = None

        self.dx = np.zeros(3)

        self.ee_pose = Pose()

        self.robot = Robot()

        self.curr_time = None

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

        if self.curr_time is not None:
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

    def set_controller_params(self, k0, q):
        self.k0 = k0
        self.q = q

    def get_ee_position(self):
        return self.ee_pose.position

    def set_ee_vel(self, dx):
        self.dx = dx

    def get_joint_position(self):
        return self.q_.tolist()
