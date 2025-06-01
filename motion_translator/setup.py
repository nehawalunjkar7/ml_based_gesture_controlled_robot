from setuptools import setup
import os
from glob import glob

package_name = 'motion_translator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # This is your Python module
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))  # Include launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shubham',
    maintainer_email='your@email.com',
    description='Translates gesture commands into velocity for robot control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_translator_node = motion_translator.motion_translator_node:main',
        ],
    },
)
