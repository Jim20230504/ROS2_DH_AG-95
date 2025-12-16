from setuptools import find_packages
from setuptools import setup

setup(
    name='dh_gripper_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('dh_gripper_msgs', 'dh_gripper_msgs.*')),
)
