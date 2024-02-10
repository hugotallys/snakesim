from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    k_values = [0.0, 0.05, 0.1, 0.2]

    for k0 in k_values:
        node = Node(
            package='snakesim_control',
            executable='resolved_rate_control',
            name='resolved_rate_controller',
            parameters=[{
                'k0': k0, 'metric': 'manipulability',
                'dx': -6.0, 'dy': .5, 'q0': [30., -30., -30.]
            }],
            output='screen'
        )
        ld.add_action(node)

    for k0 in k_values:
        node = Node(
            package='snakesim_control',
            executable='resolved_rate_control',
            name='resolved_rate_controller',
            parameters=[{
                'k0': 10. * k0, 'metric': 'jointLimit',
                'dx': -3.0, 'dy': -.5, 'q0': [60., -60., -60.]
            }],
            output='screen'
        )
        ld.add_action(node)

    return ld
