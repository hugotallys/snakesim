import time

import numpy as np
import rclpy

from rclpy.node import Node
from roboticstoolbox import DHRobot, RevoluteDH
from sensor_msgs.msg import JointState
from spatialmath import SE3

from geometry_msgs.msg import Twist, Point
from snakesim_interfaces.srv import SetJointState
from snakesim_interfaces.msg import InputRRC, OutputRRC


class Robot:
    def __init__(self):

        self.qlim = [-np.pi * 0.5, np.pi * 0.5]

        self.T01 = self.dh_transform(
            d=0.04, a=0.0, alpha=np.pi / 2, theta=-np.pi / 2
        )

        self.robot = DHRobot(
            [
                RevoluteDH(
                    d=0,
                    a=0.06,
                    alpha=np.pi / 2,
                    qlim=self.qlim,
                    offset=np.pi / 2,
                ),
                RevoluteDH(d=0, a=0.06, alpha=-np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=-np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.02, alpha=np.pi / 2, qlim=self.qlim),
            ],
            name="Snake",
        )

    def dh_transform(self, d, a, alpha, theta):
        dh = np.array(
            [
                [
                    np.cos(theta),
                    -np.sin(theta) * np.cos(alpha),
                    np.sin(theta) * np.sin(alpha),
                    a * np.cos(theta),
                ],
                [
                    np.sin(theta),
                    np.cos(theta) * np.cos(alpha),
                    -np.cos(theta) * np.sin(alpha),
                    a * np.sin(theta),
                ],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1],
            ]
        )
        return SE3(dh)

    def jacobian(self, q):
        return self.robot.jacob0(q, half="trans")[:3, :]

    def q0dot(self, q, k0=0.0, metric="joint_distance"):
        if metric == "joint_distance":
            n = len(q)
            values = []
            qlims = self.robot.qlim.T
            for i in range(qlims.shape[0]):
                q_min, q_max = qlims[i][0], qlims[i][1]
                q_mean = 0.5 * (q_max + q_min)
                values.append((q[i] - q_mean) / (q_max - q_min) ** 2)
            return (-k0 / n) * (np.array(values)).reshape(n, 1)
        elif metric == "manipulability":
            return k0 * self.gradient(self.manipulability, q)
        else:
            raise ValueError(
                f"Invalid metric: {metric}. Must be either"
                " 'joint_distance' or 'manipulability'."
            )

    def metric(self, q, name=None):
        if name is None:
            return 0.0
        q = np.array(q)
        if name == "joint_distance":
            n = len(q)
            values = []
            qlims = self.robot.qlim.T
            for i in range(qlims.shape[0]):
                q_min, q_max = qlims[i][0], qlims[i][1]
                q_mean = 0.5 * (q_max + q_min)
                values.append(((q[i] - q_mean) / (q_max - q_min)) ** 2)
            return (-1.0 / (2 * n)) * np.sum(values)
        elif name == "manipulability":
            return self.manipulability(q)
        else:
            raise ValueError(
                f"Invalid metric: {name}. Must be either"
                " 'joint_distance' or 'manipulability'."
            )

    def manipulability(self, q):
        J = self.jacobian(q)
        return np.sqrt(np.linalg.det(J @ J.T))

    @staticmethod
    def gradient(f, x, h=0.01):
        grad = np.zeros_like(x)
        for i in range(len(x)):
            xp, xm = x.copy(), x.copy()
            xp[i] = xp[i] + h
            xm[i] = xm[i] - h
            grad[i] = (f(xp) - f(xm)) / (2 * h)
        return grad

    def get_fkine_position(self, q):
        q = np.array(q)
        T = self.T01 @ self.robot.fkine(q)
        return T.t

    def update_joint_position(self, q, dx, k0, dt, metric_name):
        q = np.array(q)
        dx = self.T01.R.T @ dx

        J = self.jacobian(q)
        JT = np.linalg.pinv(J)

        q0dot = self.q0dot(q, k0=k0, metric=metric_name).reshape(-1, 1)

        dq = (JT @ dx).reshape(-1, 1) + (
            np.eye(self.robot.n) - JT @ J
        ) @ q0dot

        return np.clip(q + dq.flatten() * dt, *self.qlim).tolist()


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")

        self.joint_position = None
        self.robot = Robot()

        self.joint_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 10
        )

        self.joint_pub = self.create_publisher(
            JointState, "target_joint_states", 10
        )

        self.create_timer(0.032, self.publish_joint_state)

        self.set_joint_service = self.create_service(
            SetJointState,
            "set_joint_state",
            self.set_joint_state_callback,
        )

        self.twist = Twist()
        self.gain = None
        self.metric_name = None

        self.in_rrc_msg_sub = self.create_subscription(
            InputRRC, "rrc_input", self.rrc_msg_callback, 10
        )

        self.out_rrc_msg_pub = self.create_publisher(
            OutputRRC, "rrc_output", 10
        )

        self.create_timer(0.032, self.publish_rrc_output)

        self.controller_enabled = False

    def rrc_msg_callback(self, msg):
        self.twist = msg.twist
        self.gain = msg.gain
        self.metric_name = msg.metric_name

    def joint_state_callback(self, msg):
        self.joint_position = msg.position

    def set_joint_state_callback(self, request, response):

        self.controller_enabled = False

        time.sleep(1)

        msg = JointState()
        msg.name = [f"rotationalMotor{i+1}" for i in range(5)]
        msg.position = request.joint_states.position

        self.joint_pub.publish(msg)

        self.joint_position = msg.position

        time.sleep(1)

        response.success = True

        self.controller_enabled = True

        return response

    def publish_joint_state(self):
        msg = JointState()
        msg.name = [f"rotationalMotor{i+1}" for i in range(5)]

        twist_arr = np.array(
            [
                self.twist.linear.x,
                self.twist.linear.y,
                self.twist.linear.z,
            ]
        )

        if self.controller_enabled:
            if self.metric_name is not None and self.gain is not None:
                msg.position = self.robot.update_joint_position(
                    q=self.joint_position,
                    dx=twist_arr,
                    k0=self.gain,
                    dt=0.032,
                    metric_name=self.metric_name,
                )

                self.joint_pub.publish(msg)

    def publish_rrc_output(self):
        msg = OutputRRC()

        if self.joint_position is not None:
            msg.score = self.robot.metric(
                self.joint_position, self.metric_name
            )
            pos = self.robot.get_fkine_position(self.joint_position)
            msg.end_effector = Point(x=pos[0], y=pos[1], z=pos[2])
            self.out_rrc_msg_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = RobotController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
