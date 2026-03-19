from setuptools import setup

setup(
    name='language_node',
    version='0.1.0',
    packages=['language_node'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/language_node']),
        ('share/language_node', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'language_node = language_node.language_node:main',
        ],
    },
)
