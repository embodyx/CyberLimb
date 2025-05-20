from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'sensory_input'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yml')),
        ('lib/' + package_name, []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jake',
    maintainer_email='jakek927@uw.edu',
    description='Sensory input package for cyberlimb',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensory_input_node = sensory_input.sensory_input_node:main',
        ],
    },
) 