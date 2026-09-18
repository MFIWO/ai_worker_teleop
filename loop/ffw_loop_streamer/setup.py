import glob
import os

from setuptools import find_packages, setup

package_name = 'ffw_loop_streamer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tony Lee',
    maintainer_email='tony@config.inc',
    description='Config Loop streamer sidecar for the ROBOTIS AI Worker.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loop_streamer = ffw_loop_streamer.loop_streamer_node:main',
        ],
    },
)
