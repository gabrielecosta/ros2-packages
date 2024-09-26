from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('msg/*.msg')),  # Include i messaggi personalizzati
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriele',
    maintainer_email='gabrielenicolocosta.90@gmail.com',
    description='Aruco detector node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arucodetector = aruco_detector.aruco_detector_node:main',
            'arucodisplayer = aruco_detector.aruco_displayer:main',
        ],
    },
)
