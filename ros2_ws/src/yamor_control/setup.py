from setuptools import setup

package_name = 'yamor_control'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/yamor.wbt']))
data_files.append(('share/' + package_name + '/protos', ['protos/Yamor.proto', 'protos/YamorBase.proto']))
data_files.append(('share/' + package_name + '/resource', [f'resource/yamor-{i}.urdf' for i in range(1, 8)]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yamor_driver = yamor_control.yamor_robot_driver:main',
            'yamor_controller = yamor_control.resolved_rate_controller:main',
        ],
    },
)
