import rclpy

from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time

N_JOINTS = 5


class SnakeDriver:

    def init(self, webots_node, properties):

        self.__robot = webots_node.robot

        self.motors = [
            self.__robot.getDevice(f"rotationalMotor{i}")
            for i in range(1, N_JOINTS + 1)
        ]

        self.sensors = [
            self.get_sensor_device(f"positionSensor{i}")
            for i in range(1, N_JOINTS + 1)
        ]

        rclpy.init(args=None)

        self.__node = rclpy.create_node("snake_driver")

        self.joint_position = None

        self.joint_pub = self.__node.create_publisher(
            JointState, "joint_states", 10
        )

        self.joint_sub = self.__node.create_subscription(
            JointState, "target_joint_states", self.joint_callback, 10
        )

    def get_sensor_device(self, name, sampling_period=32):
        sensor = self.__robot.getDevice(name)
        sensor.enable(sampling_period)
        return sensor

    def joint_callback(self, msg):
        self.joint_position = msg.position

    def publish_joint_state(self, joint_position):
        msg = JointState()
        now = self.__robot.getTime()
        sec = int(now)
        nsec = int((now - sec) * 1e9)
        msg.header.stamp = Time(sec=sec, nanosec=nsec)
        msg.name = [f"rotationalMotor{i+1}" for i in range(N_JOINTS)]
        msg.position = joint_position
        self.joint_pub.publish(msg)

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        if self.joint_position is not None:
            for i, value in enumerate(self.joint_position):
                self.motors[i].setPosition(value)

        read_joint_position = [
            self.sensors[i].getValue() for i in range(N_JOINTS)
        ]

        self.publish_joint_state(read_joint_position)
