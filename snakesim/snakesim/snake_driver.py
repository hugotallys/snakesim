import rclpy
import numpy as np

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray


N_JOINTS = 7


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

        self.__target_position = Float64MultiArray()
        self.end_eff = self.__robot.getFromDef("END_EFF")

        rclpy.init(args=None)

        self.__node = rclpy.create_node("snake_driver")

        self.__position_publisher = self.__node.create_publisher(
            Float64MultiArray, "position", 10
        )
        self.__end_eff_pose_publisher = self.__node.create_publisher(
            Pose, "endEffPose", 10
        )

        self.__node.create_subscription(
            Float64MultiArray,
            "targetPosition",
            self.__position_callback,
            10,
        )

    def get_sensor_device(self, name, sampling_period=16):
        sensor = self.__robot.getDevice(name)
        sensor.enable(sampling_period)
        return sensor

    @staticmethod
    def rotation_matrix_to_quaternion(r):

        q_w = np.sqrt(1 + r[0, 0] + r[1, 1] + r[2, 2]) / 2.0
        q_x = (r[2, 1] - r[1, 2]) / (4 * q_w)
        q_y = (r[0, 2] - r[2, 0]) / (4 * q_w)
        q_z = (r[1, 0] - r[0, 1]) / (4 * q_w)

        return [q_x, q_y, q_z, q_w]

    def get_end_effector_pose(self):

        eff_pose = np.array(self.end_eff.getPose()).reshape(4, 4)
        orientation = self.rotation_matrix_to_quaternion(
            eff_pose[:-1, :-1]
        )

        pose_msg = Pose()

        pose_msg.position.x = eff_pose[0, 3]
        pose_msg.position.y = eff_pose[1, 3]
        pose_msg.position.z = eff_pose[2, 3]

        pose_msg.orientation.x = orientation[0]
        pose_msg.orientation.y = orientation[1]
        pose_msg.orientation.z = orientation[2]
        pose_msg.orientation.w = orientation[3]

        return pose_msg

    def __position_callback(self, position):
        self.__target_position = position

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        for i, value in enumerate(self.__target_position.data):
            self.__motors[i].setPosition(value)

        sensor_data = [
            self.__sensors[i].getValue() for i in range(N_JOINTS)
        ]

        self.__position_publisher.publish(
            msg=Float64MultiArray(data=sensor_data)
        )

        self.__end_eff_pose_publisher.publish(
            msg=self.get_end_effector_pose()
        )
