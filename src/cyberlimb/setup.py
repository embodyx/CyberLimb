from setuptools import setup
import os
from glob import glob

package_name = 'cyberlimb'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jake',
    maintainer_email='jakek927@uw.edu',
    description='Cyberlimb ROS 2 launch package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'some_node = cyberlimb.some_node:main',
        ],
    },
)
