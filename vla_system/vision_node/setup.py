#!/usr/bin/env python3
# vision_node/setup.py
from setuptools import setup

setup(
    name='vision_node',
    version='0.1.0',
    packages=['vision_node'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/vision_node']),
        ('share/vision_node', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'vision_node = vision_node.vision_node:main',
        ],
    },
)
