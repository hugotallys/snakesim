from setuptools import setup

package_name = 'snakesim'

data_files = []

data_files.append((
    'share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append((
    'share/' + package_name + '/launch', ['launch/snake_launch.py']))
data_files.append((
    'share/' + package_name + '/worlds', ['worlds/snake.wbt']))

stl_files = ['servo.stl', 'socket.stl', 'servoSocket.stl']

data_files.append((
    'share/' + package_name + '/protos/meshes',
    [f'protos/meshes/{fname}' for fname in stl_files]
))
data_files.append((
    'share/' + package_name + '/resource', ['resource/snake.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hgtllys',
    maintainer_email='htmo@ic.ufal.br',
    description='Simulation of a snake-like robot using Webots and ROS2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'snake_driver = snakesim.snake_driver:main',
            'snake_control = snakesim.resolved_rate_controller:main',
        ],
    },
)
