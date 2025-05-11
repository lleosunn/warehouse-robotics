from setuptools import setup

package_name = 'car_follower'

setup(
    name=package_name,
    version='2.4.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/car_follower.launch.py']),        
     ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Richard Chen',
    maintainer_email='rachen@mit.edu',
    author='Richard Chen, James Shaw, Eve Silfanus',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Depth-based car follower node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_follower_node = car_follower.depth_follower_node:main'
        ],
    },
)
