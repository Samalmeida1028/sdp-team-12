from setuptools import find_packages
from setuptools import setup

setup(
    name='aruco_msgs',
    version='2.0.0',
    packages=find_packages(
        include=('aruco_msgs', 'aruco_msgs.*')),
)
