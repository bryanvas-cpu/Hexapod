from setuptools import find_packages
from setuptools import setup

setup(
    name='hexapod_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('hexapod_interfaces', 'hexapod_interfaces.*')),
)
