from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['ur_icam'],
    package_dir={'': 'ur_icam_description/src'}
)
setup(**setup_args)
