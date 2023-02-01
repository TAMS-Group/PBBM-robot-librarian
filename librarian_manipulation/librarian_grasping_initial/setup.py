from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['librarian_grasping_initial'],
    package_dir={'':'src'}
)

setup(**d)
