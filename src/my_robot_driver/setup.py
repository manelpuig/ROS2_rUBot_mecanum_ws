from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='manel.puig@ub.edu',
    description='ROS2 package for controlling a Rubot with mecanum wheels',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rubot_mecanum_driver_node = my_robot_driver.rubot_mecanum_driver:main',
        ],
    },
)
