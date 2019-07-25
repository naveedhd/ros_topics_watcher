#!/usr/bin/env python

from setuptools import find_packages, setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=find_packages(exclude=['tests*', 'docs*', 'launch*']),
    # global installs, for package relative installs, look in CMakeLists.txt
    keywords=['ros', 'topics', 'introspection'],
    scripts=['scripts/ros-topics-watcher']
)

setup(**d)
