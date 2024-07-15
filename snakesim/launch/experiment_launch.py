from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="snakesim",
                executable="snake_trajectory",
                name="snake_trajectory",
                output="screen",
            ),
            Node(
                package="snakesim",
                executable="snake_trajectory_client",
                name="snake_trajectory_client",
                output="screen",
            ),
        ]
    )
