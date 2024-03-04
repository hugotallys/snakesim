import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock

from roboticstoolbox import DHRobot, RevoluteDH


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


class ResolvedRateController(Node):
    def __init__(self):
        super().__init__("resolved_rate_controller")
        self.n_joints = 7

        self.publisher_ = self.create_publisher(
            Float64MultiArray, "targetPosition", 10
        )

        self.subscriber_ = self.create_subscription(
            Float64MultiArray, "position", self.position_callback, 10
        )

        self.clock_subscriber_ = self.create_subscription(
            Clock, "clock", self.simtime_callback, 10
        )

        self.get_logger().info("*** Resolved-Rate-Controller started ***")
        _angle = np.deg2rad(60)
        self.q = np.array([_angle, 0, -_angle, 0, -_angle, 0, 0])
        self.q_ = self.q.copy()

        self.dx = np.zeros(3)

        self.robot = Robot()

        self.set_joint_position()

        self.get_logger().info("Starting the control loop...")

        self.curr_time = 0.0
        self.start_time = None

    def set_joint_position(self):
        msg = Float64MultiArray(data=self.q.tolist())
        self.publisher_.publish(msg)

    def position_callback(self, msg):
        self.q_ = np.array(msg.data)

    def simtime_callback(self, msg):
        simulation_time = msg.clock.sec + msg.clock.nanosec / 1e9

        if self.start_time is None:
            self.start_time = simulation_time

        if simulation_time - self.start_time > 5.0:
            dt = simulation_time - self.curr_time

            J = self.robot.jacobian(self.q_)
            JT = np.linalg.pinv(J)

            q0dot = self.robot.q0dot(self.q_, k0=0.0)

            dq = (JT @ self.dx).reshape(-1, 1) + (
                np.eye(self.n_joints) - JT @ J
            ) @ q0dot

            self.q = np.clip(self.q_ + dq.flatten() * dt, -3.14, 3.14)

            a = 0.1
            vy = -a * np.sin(simulation_time - 5.0)
            vz = 0.1  # a*np.cos(simulation_time - 5.0)
            vx = -0.2  # vy*vz
            self.dx = np.array([vx, vy, vz])

        self.set_joint_position()
        self.curr_time = simulation_time


def main(args=None):
    rclpy.init(args=args)
    node = ResolvedRateController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
