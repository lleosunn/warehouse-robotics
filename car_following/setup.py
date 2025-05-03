from setuptools import setup

package_name = 'car_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/car_follower.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Depth-based car follower node for DJI RoboMaster using ROS 2 Humble',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_follower_node = car_follower.depth_follower_node:main',
        ],
    },
)
