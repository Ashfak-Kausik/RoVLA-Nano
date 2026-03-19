from setuptools import setup

setup(
    name='action_node',
    version='0.1.0',
    packages=['action_node'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/action_node']),
        ('share/action_node', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'action_node = action_node.action_node:main',
        ],
    },
)
