from setuptools import setup, find_packages

package_name = 'car_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cone_controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yiyi',
    maintainer_email='yiyi@ucsd.edu',
    description='Low-level velocity controller for a unicycle-like robot (ROS 2)',
    license='TODO',
    entry_points={
        'console_scripts': [
            'cone_controller = car_control.cone_controller_ros2_node:main',
        ],
    },
)
