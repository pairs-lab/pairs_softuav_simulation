## Makes the ROS-agnostic library under src/pairs_softuav_sim importable.
## Invoked by catkin_python_setup() in CMakeLists.txt — do not run directly.
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["pairs_softuav_sim"],
    package_dir={"": "src"},
)

setup(**setup_args)
