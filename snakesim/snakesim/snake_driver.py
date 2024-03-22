import rclpy
import numpy as np

from geometry_msgs.msg import Pose, Twist
from .robot import Robot

from snakesim_interfaces.srv import JointState
from std_msgs.msg import Float64

N_JOINTS = 5
RAD_120 = np.deg2rad(120)


class SnakeDriver:

    def init(self, webots_node, properties):

        self.__robot = webots_node.robot

        self.__motors = [
            self.__robot.getDevice(f"rotationalMotor{i}")
            for i in range(1, N_JOINTS + 1)
        ]

        self.__sensors = [
            self.get_sensor_device(f"positionSensor{i}")
            for i in range(1, N_JOINTS + 1)
        ]

        self.end_eff = self.__robot.getFromDef("END_EFFECTOR")

        rclpy.init(args=None)

        self.__node = rclpy.create_node("snake_driver")

        self.__end_eff_pose_publisher = self.__node.create_publisher(
            Pose, "end_effector_pose", 10
        )

        self.target_twist = Twist()

        self.__metric_publisher = self.__node.create_publisher(
            Float64, "metric", 10
        )

        self.__target_twist_subscriber = self.__node.create_subscription(
            Twist, "target_twist", self.twist_callback, 10
        )

        self.__target_gain_subscriber = self.__node.create_subscription(
            Float64, "target_gain", self.gain_callback, 10
        )

        self.__init_joint_state_service = self.__node.create_service(
            JointState,
            "init_joint_state",
            self.init_joint_state_callback,
        )

        self.robot = Robot()
        self.gain = 0.0
        self.metric_name = "manipulability"

    def get_sensor_device(self, name, sampling_period=16):
        sensor = self.__robot.getDevice(name)
        sensor.enable(sampling_period)
        return sensor

    def get_end_effector_pose(self):

        eff_pose = np.array(self.end_eff.getPose()).reshape(4, 4)

        pose_msg = Pose()

        pose_msg.position.x = eff_pose[0, 3]
        pose_msg.position.y = eff_pose[1, 3]
        pose_msg.position.z = eff_pose[2, 3]

        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 1.0
        pose_msg.orientation.w = 0.0

        return pose_msg

    def twist_callback(self, msg):
        self.target_twist = msg

    def gain_callback(self, msg):
        self.gain = msg.data

    def delay_simulation(self, seconds):
        n_iter = int(1000 * seconds / self.__robot.getBasicTimeStep())
        for _ in range(n_iter):
            self.__robot.step()

    def init_joint_state_callback(self, request, response):
        for i, value in enumerate(request.joint_state):
            self.__motors[i].setPosition(value)

        self.delay_simulation(1)

        response.success = True

        return response

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        dx = np.array(
            [
                self.target_twist.linear.x,
                self.target_twist.linear.y,
                self.target_twist.linear.z,
            ]
        )

        read_joint_position = np.array(
            [self.__sensors[i].getValue() for i in range(N_JOINTS)]
        )

        self.__metric_publisher.publish(
            Float64(
                data=self.robot.metric(
                    read_joint_position, name=self.metric_name
                )
            )
        )

        target_joint_position = self.update_joint_position(
            read_joint_position, dx, self.gain
        )

        for i, value in enumerate(target_joint_position):
            self.__motors[i].setPosition(value)

        self.__end_eff_pose_publisher.publish(self.get_end_effector_pose())

    def update_joint_position(self, q, dx, k0=0.0, dt=0.032):

        J = self.robot.jacobian(q)
        JT = np.linalg.pinv(J)

        q0dot = self.robot.q0dot(
            q, k0=k0, metric=self.metric_name
        ).reshape(-1, 1)

        dq = (JT @ dx).reshape(-1, 1) + (np.eye(N_JOINTS) - JT @ J) @ q0dot

        return np.clip(q + dq.flatten() * dt, -RAD_120, RAD_120)
