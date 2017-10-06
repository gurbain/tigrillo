"""
Install Tigrillo main libraries
"""

from setuptools import setup
from setuptools import find_packages


setup(name='tigrillo',
      version='1.0',
      description='A library to simulate and pilot the Tigrillo quadruped robot',
      author='Gabriel Urbain',
      author_email='gabriel.urbain@ugent.be',
      url='https://github.com/gabs48/tigrillo',
      license='MIT',
      install_requires=[
            'scipy>=0.19.1',
            'numpy',
            'matplotlib',
            'GitPython',
            'pause>=0.1.2',
            'lxml>=3.4.0',
            'mpi4py>=1.3.1',
            'psutil>=3.3.0',
            'pybullet>=1.0.7',
            "PyYaml",
      ],
      extras_require={
            'pyserial': ['scipy>=3.0.1'],
            'rospy': ['rospy>=1.12.7','rospkg', 'catkin_pkg'],
      },
      packages=find_packages())
