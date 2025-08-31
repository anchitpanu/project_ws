from setuptools import find_packages
from setuptools import setup

setup(
    name='quin_core',
    version='2.1.5',
    packages=find_packages(
        include=('quin_core', 'quin_core.*')),
)
