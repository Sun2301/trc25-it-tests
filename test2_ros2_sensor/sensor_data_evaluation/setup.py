from setuptools import setup
from glob import glob
import os

package_name = 'sensor_data_evaluation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sun',
    maintainer_email='sundaybernkpokpo@gmail.com',
    description='ROS2 package for TRC25 test 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
    python_requires='>=3.6',
)
