import os
import launch

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory("snakesim")
    robot_desc_path = os.path.join(package_dir, "resource", "snake.urdf")

    webots = WebotsLauncher(
        world=os.path.join(package_dir, "worlds", "snake.wbt"),
        ros2_supervisor=True,
    )

    snake_driver = WebotsController(
        robot_name="Snake",
        parameters=[
            {"robot_description": robot_desc_path},
        ],
    )

    return LaunchDescription(
        [
            webots,
            webots._supervisor,
            snake_driver,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[
                        launch.actions.EmitEvent(
                            event=launch.events.Shutdown()
                        )
                    ],
                )
            ),
            Node(
                package="snakesim",
                executable="snake_trajectory",
                name="snake_trajectory",
                output="screen",
            ),
        ]
    )
