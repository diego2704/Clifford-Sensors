#!/usr/bin/env python3
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tesis_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name+ '/launch',glob('tesis_launch/launch/*')),
        #('share/' + package_name + '/launch', glob('tesis_launch/*.launch.py')),
        #('share/' + package_name + '/launch', glob('tesis_launch/launch/*.py')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cliford',
    maintainer_email='cliford@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coman_pub =  tesis_launch.comandos_pub:main',
            'coman_sus =  tesis_launch.comandos_sus:main',
            'cama_pub = tesis_launch.camaron_pub:main',
            'cama_sus = tesis_launch.camaron_sus:main',
            'rplidar.launch = tesis_launch.rplidar.launch:main',
            'rviz_launcher_node = tesis_launch.rviz_launcher_node:main',
        ],
    },
)
