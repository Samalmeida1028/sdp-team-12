from setuptools import find_packages
from setuptools import setup

setup(
    name='basic_mobile_robot',
    version='0.0.0',
    packages=find_packages(
        include=('basic_mobile_robot', 'basic_mobile_robot.*')),
)
