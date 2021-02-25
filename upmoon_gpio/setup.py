from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['upmoon_gpio'],
    package_dir={'': 'src'},
    install_requires=['pysabertooth', 'RPI.GPIO', 'PyTrinamic']
)

setup(**setup_args)