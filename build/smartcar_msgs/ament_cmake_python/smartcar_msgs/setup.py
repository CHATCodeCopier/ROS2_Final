from setuptools import find_packages
from setuptools import setup

setup(
    name='smartcar_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('smartcar_msgs', 'smartcar_msgs.*')),
)
