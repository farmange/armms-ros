from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['armms_user_intent'],
    # scripts=['bin/myscript'],
    package_dir={'': 'scripts'}
)

setup(**d)
