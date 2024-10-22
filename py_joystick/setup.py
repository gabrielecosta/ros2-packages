from setuptools import find_packages, setup

package_name = 'py_joystick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriele',
    maintainer_email='gabrielenicolocosta.90@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'writingnode = py_joystick.writing:main',
            'readingnode = py_joystick.reading:main',
        ],
    },
)
