#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['ros_arduino_python'],
    package_dir={'': 'src'},
    )

setup(**d)
