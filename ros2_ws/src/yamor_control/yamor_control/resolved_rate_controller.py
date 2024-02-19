import rclpy
import time
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float32

from functools import partial
from roboticstoolbox import DHRobot, RevoluteDH


class Robot:
    def __init__(self):
        self.robot = DHRobot([
            RevoluteDH(d=0, a=0.097, alpha=0),
            RevoluteDH(d=0, a=0.097, alpha=0),
            RevoluteDH(d=0, a=0.097, alpha=0),
            RevoluteDH(d=0, a=0.097, alpha=0),
            RevoluteDH(d=0, a=0.097, alpha=0),
            RevoluteDH(d=0, a=0.097, alpha=0),
            RevoluteDH(d=0, a=0.097, alpha=0)
        ], name="YamorArm")
        self.qlims = [
            (-np.pi*0.5, np.pi*0.5) for _ in range(7)
        ]

    def jacobian(self, q):
        return self.robot.jacob0(q, half="trans")[:2, :]

    def q0dot(self, q, k0=1.0):
        n = len(q)
        values = []
        for i in range(len(self.qlims)):
            q_min, q_max = self.qlims[i][0], self.qlims[i][1]
            q_mean = 0.5*(q_max + q_min)
            values.append(
                (q[i] - q_mean) / (q_max - q_min)**2
            )
        return (-k0 / n) * (np.array(values)).reshape(n, 1)


class ResolvedRateController(Node):
    def __init__(self):
        super().__init__('resolved_rate_controller')
        self.__n_topics = 7
        self.publishers_ = [
            self.create_publisher(
                Float32, f'targetPosition{i}', 10
            ) for i in range(1, self.__n_topics + 1)
        ]

        joint_topics = [f'position{i}' for i in range(1, self.__n_topics + 1)]

        # Create subscribers for each joint topic
        self.subscribers_ = []
        for topic in joint_topics:
            callback_func = partial(
                self.position_callback, topic=topic)
            subscriber = self.create_subscription(
                Float32, topic, callback_func, 10)
            self.subscribers_.append(subscriber)

        self.dt = 0.1
        self.delta_t = 0.

        self.timer_ = self.create_timer(self.dt, self.publish_message)
        self.get_logger().info("*** Resolved-Rate-Controller started ***")
        _angle = np.deg2rad(60)
        self.q = np.array(
            [_angle, 0, -_angle, 0, -_angle, 0, 0])
        self.q_ = self.q.copy()
        self.dx = np.array([-0.1, 0.05])

        self.robot = Robot()

        self.set_joint_position()

        # halt and wait for 3s
        self.get_logger().info("Waiting for 3s...")

        time.sleep(3)

        self.get_logger().info("Starting the control loop...")

    def set_joint_position(self):
        for i, q_value in enumerate(self.q):
            msg = Float32(data=q_value)
            self.publishers_[i].publish(msg)

    def publish_message(self):
        J = self.robot.jacobian(self.q_)
        JT = np.linalg.pinv(J)

        q0dot = self.robot.q0dot(self.q_, k0=25.0)

        dq = (JT @ self.dx).reshape(-1, 1) + (
            np.eye(self.__n_topics) - JT @ J) @ q0dot

        self.q = np.clip(self.q_ + dq.flatten() * self.dt, -1.57, 1.57)

        self.set_joint_position()

    def position_callback(self, msg, topic):
        # Extract the joint index from the topic name
        joint_index = int(topic.replace("position", "")) - 1
        self.q_[joint_index] = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = ResolvedRateController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
