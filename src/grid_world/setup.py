from setuptools import find_packages, setup
import os

package_name = 'grid_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['grid_world/grid_world_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alijnadi',
    maintainer_email='a.jnadi@innopolis.university',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'grid_world_node = grid_world.world_node:main',
            'grid_world_solver = grid_world.command_node:main'
        ],
    },
)

# TODO: Write names of the group memebers.
# Dmitriy Ezovskikh