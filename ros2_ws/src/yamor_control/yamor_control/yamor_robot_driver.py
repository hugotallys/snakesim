import rclpy

from std_msgs.msg import Float32


class YamorDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.__motor = self.__robot.getDevice('motor')
        self.__position_sensor = self.__motor.getPositionSensor()

        self.__target_position = Float32()

        self.__yamorCount = properties["yamorCount"]

        rclpy.init(args=None)
        self.__node = rclpy.create_node('yamor_driver')
        self.__node.create_subscription(
            Float32, f'targetPosition{self.__yamorCount}',
            self.__position_callback, 1)

        self.__position_sensor.enable(1)
        self.__position_publisher = self.__node.create_publisher(
            Float32, f'position{self.__yamorCount}', 1)

    def __position_callback(self, position):
        self.__target_position = position

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.__motor.setPosition(self.__target_position.data)
        self.__position_publisher.publish(
            Float32(data=self.__position_sensor.getValue()))
