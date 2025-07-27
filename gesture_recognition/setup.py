from setuptools import setup
import os
from glob import glob

package_name = 'gesture_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={
        package_name: ['*.joblib'],  # ← Add this line
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shubham',
    maintainer_email='your@email.com',
    description='Gesture recognition module using webcam',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_node_v1 = gesture_recognition.gesture_node_v1:main',
            'gesture_node_v2 = gesture_recognition.gesture_node_v2:main',
            'gesture_node_ML_v3 = gesture_recognition.gesture_node_ML_v3:main',
        ],
    },
)
