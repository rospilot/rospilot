# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rospilot', 'rospilot.assets', 'vlc_server'],
    package_dir={'': 'src'},
    package_data={'rospilot.assets': ['*', '**/*']},
    requires=['std_msgs', 'rospy']
)

setup(**setup_args)
