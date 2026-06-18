from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["flight_safety", "flight_safety.diagnosis"],
    package_dir={"": "src"},
)
setup(**setup_args)
