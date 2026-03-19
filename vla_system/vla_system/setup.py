from setuptools import setup
import os
from glob import glob

setup(
    name='vla_system',
    version='0.1.0',
    packages=['vla_system'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/vla_system']),
        ('share/vla_system', ['package.xml']),
        ('share/vla_system/launch', glob('../launch/*.py')),
        ('share/vla_system/config', glob('../config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'vla_cli = vla_system.cli:main',
        ],
    },
)
