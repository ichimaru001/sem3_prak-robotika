import os
from glob import glob
from setuptools import setup

package_name = 'lab_motion'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='Package for Modul 6: Differential-Drive Kinematics and Motion Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop = lab_motion.open_loop:main',
            'open_loop_controller = lab_motion.open_loop:main',
            'closed_loop = lab_motion.closed_loop:main',
            'closed_loop_controller = lab_motion.closed_loop:main',
            'challenge = lab_motion.challenge_controller:main',
            'challenge_controller = lab_motion.challenge_controller:main',
            'mock_turtlebot3 = lab_motion.mock_turtlebot3:main',
        ],
    },
)
