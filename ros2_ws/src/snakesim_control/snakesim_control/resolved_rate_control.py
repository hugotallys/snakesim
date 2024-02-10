import os
import rclpy
import pandas as pd
import numpy as np

from rclpy.node import Node
from snakesim_interfaces.msg import JointState
from snakesim_interfaces.srv import SpawnSnake

from roboticstoolbox import DHRobot, RevoluteDH, ctraj
from spatialmath import SE3


class PlanarArm:
    def __init__(self, arm_lengths):
        self.arm_lengths = arm_lengths
        self.N = len(arm_lengths)
        self.robot = DHRobot([
            RevoluteDH(d=0, a=length, alpha=0) for length in self.arm_lengths
        ], name=f"Planar {self.arm_lengths}R Robot")
        self.qlims = [
            (-np.pi*0.95, np.pi*0.95) for length in self.arm_lengths
        ]

    def jacobian(self, q):
        return self.robot.jacob0(q, half="trans")[:2, :]

    def q0dot(self, q, k0=1.0, objective="manipulability"):
        if objective == "manipulability":
            return k0 * self.gradient(self.manipulability, q)
        elif objective == "jointLimit":
            n = len(q)
            values = []
            for i in range(len(self.qlims)):
                q_min, q_max = self.qlims[i][0], self.qlims[i][1]
                q_mean = 0.5*(q_max + q_min)
                values.append(
                    (q[i] - q_mean) / (q_max - q_min)**2
                )
            return (-k0 / n) * (np.array(values)).reshape(n, 1)
        else:
            raise ValueError(
                "Invalid objective. Must be either"
                " 'manipulability' or 'joint_limit_distance'"
            )

    def joint_distance(self, q):
        n = len(q)
        values = []
        for i in range(len(self.qlims)):
            q_min, q_max = self.qlims[i][0], self.qlims[i][1]
            q_mean = 0.5*(q_max + q_min)
            values.append(
                ((q[i] - q_mean) / (q_max - q_min))**2
            )
        return (-1/(2*n)) * np.sum(values)

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


class ResolvedRateController(Node):
    def __init__(self):
        super().__init__('resolved_rate_controller')
        self.publisher_ = self.create_publisher(JointState, 'joint_state', 10)
        self.client_ = self.create_client(SpawnSnake, 'spawn_snake')
        self.dt = 1 / 30
        self.timer_ = self.create_timer(self.dt, self.publish_message)
        self.get_logger().info("*** Resolved-Rate-Controller started ***")
        self.total_time = 0.

        self.planar_arm = PlanarArm(arm_lengths=[1, 1, 1])

        # Data structure to store the configurations
        self.K = 100  # Number of configurations
        self.N = len(self.planar_arm.arm_lengths)  # Number of joints
        self.q = np.zeros((self.K, self.N))
        self.qd = np.zeros((self.K, self.N))
        self.m = np.zeros(self.K)
        self.jd = np.zeros(self.K)
        self.eigen_values = np.zeros((self.K, 2))
        self.eigen_angles = np.zeros(self.K)

        # Initial configuration
        self.q[0, :] = np.array([0., 90., 90.])

        self.declare_parameter('q0', self.q[0, :].tolist())
        self.q[0, :] = self.get_parameter(
            'q0').get_parameter_value().double_array_value

        self.declare_parameter("metric", "jointLimit")
        self.metric = self.get_parameter(
            "metric").get_parameter_value().string_value

        self.declare_parameter("k0", 0.0)
        self.k0 = self.get_parameter("k0").get_parameter_value().double_value

        self.declare_parameter("dx", 0.0)
        self.dx = self.get_parameter("dx").get_parameter_value().double_value

        self.declare_parameter("dy", 0.0)
        self.dy = self.get_parameter("dy").get_parameter_value().double_value

        self.declare_parameter("GUI", False)
        self.GUI = self.get_parameter("GUI").get_parameter_value().bool_value

        # Adjusting the initial configuration
        self.q[0, :] = np.deg2rad(self.q[0, :])

        # Setting up a cartesian trajectory
        self.Ti = self.planar_arm.robot.fkine(self.q[0, :])

        # Perform the trajectory
        self.traj_idx = 0

        self.compute_trajectory(delta_x=self.dx, delta_y=self.dy)

        # Save the results
        self.save_results()

        if not self.GUI:
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.call_service()

    def call_service(self):
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = SpawnSnake.Request()
        # request.lengths = self.planar_arm.arm_lengths
        request.position = self.q[0, :].tolist()
        request.origin = [0., 0.]

        future = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info(
                'Result of spawn_snake: %s' % future.result().success)
        else:
            self.get_logger().error(
                'Exception while calling service: %r' % future.exception())

    @staticmethod
    def compute_angle(eigenvectors):
        # Calculate the angle of the semi-major axis
        # with respect to the x-axis
        angle = np.arctan2(
            eigenvectors[1, 0], eigenvectors[0, 0])
        return angle

    def compute_trajectory(self, delta_x=0, delta_y=0):

        Tf = SE3.Trans(delta_x, delta_y, 0) @ self.Ti
        traj = ctraj(self.Ti, Tf, self.K)

        for i in range(1, self.K):
            t = self.Ti.t
            dx = traj[i].t[:2] - t[:2]

            J = self.planar_arm.jacobian(self.q[i - 1, :])
            JT = np.linalg.pinv(J)

            U, S, _ = np.linalg.svd(J)
            self.eigen_values[i - 1, :] = S
            self.eigen_angles[i] = self.compute_angle(U.T)

            self.m[i - 1] = np.sqrt(np.linalg.det(J @ J.T))
            self.jd[i - 1] = self.planar_arm.joint_distance(self.q[i - 1, :])

            q0dot = np.squeeze(self.planar_arm.q0dot(
                self.q[i - 1, :], k0=self.k0,
                objective=self.metric)
            )

            self.qd[i - 1, :] = JT @ dx + (
                np.eye(self.N) - JT @ J) @ q0dot

            self.q[i, :] = self.q[i - 1, :] + self.qd[i - 1, :]
            self.Ti = self.planar_arm.robot.fkine(self.q[i, :])

        self.m[self.K - 1] = self.m[self.K - 2]
        self.qd[self.K - 1, :] = self.qd[self.K - 2, :]
        self.jd[self.K - 1] = self.jd[self.K - 2]

        self.eigen_values[self.K - 1, :] = self.eigen_values[self.K - 2, :]
        self.eigen_values[0] = self.eigen_values[1]

    def publish_message(self):
        msg = JointState(
                position=self.q[self.traj_idx, :].tolist(),
                eigen_values=self.eigen_values[self.traj_idx, :].tolist(),
                eigen_angle=self.eigen_angles[self.traj_idx]
            )

        self.publisher_.publish(msg)

        self.get_logger().info(
            "Publishing joint position = %s" % str(msg.position))
        self.get_logger().info(
            "Publishing eigen values = %s" % str(msg.eigen_values))

        self.traj_idx = (self.traj_idx + 1) % self.K

    def save_results(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        dir_path = os.path.join(dir_path, "results")

        if not os.path.exists(dir_path):
            os.makedirs(dir_path)

        file_path = os.path.join(
            dir_path, f"rrc_metric={self.metric}_k0={self.k0:.3}.csv")
        t = np.arange(0, self.K) * self.dt
        df = pd.DataFrame(
            data=np.hstack(
                (t.reshape(-1, 1), self.q, self.qd,
                 self.m.reshape(-1, 1), self.jd.reshape(-1, 1),
                 self.eigen_values)),
            columns=[
                "t"
            ] + [
                f"q{i}" for i in range(self.N)
            ] + [
                f"qd{i}" for i in range(self.N)
            ] + [
                "m"
            ] + [
                "jd"
            ] + [
                f"sig{i}" for i in range(2)
            ]
        )
        df.to_csv(file_path, index=False)


def main(args=None):
    rclpy.init(args=args)

    node = ResolvedRateController()

    if node.GUI:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
