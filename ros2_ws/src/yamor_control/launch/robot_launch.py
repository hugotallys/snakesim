import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def get_description_path(package_dir, count):
    return os.path.join(
        package_dir, 'resource', f'yamor-{count}.urdf'
    )


def generate_launch_description():
    package_dir = get_package_share_directory('yamor_control')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'yamor.wbt'),
    )

    yamor_drivers = [WebotsController(
        robot_name=f'yamor_{i}',
        parameters=[
            {'robot_description': get_description_path(package_dir, i)},
        ]
    ) for i in range(1, 8)]

    return LaunchDescription([
        webots,
        *yamor_drivers,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
