from setuptools import setup

setup(
    name='coordinator_node',
    version='0.1.0',
    packages=['coordinator_node'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/coordinator_node']),
        ('share/coordinator_node', ['package.xml']),
        ('share/coordinator_node/launch', ['../../launch/vla_full.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'coordinator_node = coordinator_node.coordinator_node:main',
            'vla_cli          = vla_cli:main',
        ],
    },
)
