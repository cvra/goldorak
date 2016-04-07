#!/usr/bin/python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['uavcan_bridge'],
    package_dir={'': 'src'},
    requires=['rospkg', 'std_msgs', 'cvra_msgs', 'rospy', 'rospkg']
)

setup(**d)
