from setuptools import find_packages, setup

package_name = 'autoturtle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ajak2k',
    maintainer_email='assrira1@uci.edu',
    description='A simple package written for the EECS221 class at UCI in Spring 2024',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_teleop_node = autoturtle.my_teleop_node:main',
            'swim_to_goal = autoturtle.swim_to_goal:main',
            'swim_node = autoturtle.swim_node:main'
        ],
    },
)
