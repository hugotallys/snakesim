import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'snakesim_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hgtllys',
    maintainer_email='htmo@ic.ufal.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "resolved_rate_control=snakesim_control.resolved_rate_control:main",
            "plot_results=snakesim_control.plot_results:main",
        ],
    },
)
