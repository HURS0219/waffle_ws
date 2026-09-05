from setuptools import setup
import os
from glob import glob

package_name = 'waffle_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='waffle user',
    maintainer_email='user@example.com',
    description='SLAM + Nav2 + three-point auto navigation for TurtleBot3 Waffle in a maze',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_follow = waffle_navigation.waypoint_follow:main',
        ],
    },
)
