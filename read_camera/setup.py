from setuptools import setup

package_name = 'read_camera'

setup(
    name=package_name,
    version='2.4.0',
    packages=[],
    py_modules=[
        'read_camera'
    ],
    # data_files=[
    #    ('share/ament_index/resource_index/packages',
    #        ['resource/' + package_name]),
    #    ('share/' + package_name, ['package.xml']),
    # ],
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
    description='A test package to read camera input',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_camera = read_camera:main'
        ],
    },
)
