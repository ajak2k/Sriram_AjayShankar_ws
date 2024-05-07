from setuptools import find_packages, setup

package_name = 'turtlebot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajak2k',
    maintainer_email='sajaax2000@gmail.com',
    description='a simple package to impliment a pid control to read the /odom topic of turtlebot3 from gazebo and control it via the /cmd_vel. The motion_planner will give the destinations to the PID controller',
    license='MIT-0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = turtlebot.pid_controller:main',
            'motion_planner = turtlebot.motion_planner:main'
        ],
    },
)
