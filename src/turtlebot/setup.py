from setuptools import find_packages, setup

package_name = 'turtlebot'

setup(
    name=package_name,
    version='0.4.0',
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
    description='A package that impliments PID control and RRT based motion planning for the turtlebot3 in gazebo. The controller gets inputs from the motion_planner',
    license='MIT-0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = turtlebot.pid_controller:main',
            'motion_planner = turtlebot.motion_planner:main',
            'map_reader_template = turtlebot.map_reader_template:main',
            'rrt = turtlebot.rrt:main',
            'rrt_node = turtlebot.rrt_node:main',
        ],
    },
)
