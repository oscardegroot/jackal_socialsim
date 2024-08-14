from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['jackal_socialsim'],
    scripts=['scripts/condition_check.py', 'scripts/data_recorder.py'],
    package_dir={'': 'scripts'}
)

setup(**d)