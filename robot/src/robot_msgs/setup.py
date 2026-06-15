## catkin-Style setup.py – wird via catkin_python_setup() aus CMakeLists.txt genutzt.
## NICHT direkt ausführen (kein setup.py install).
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['robot_msgs'],
    package_dir={'': 'src'},
)

setup(**setup_args)
