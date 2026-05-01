from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'weld_inspection_classic'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='svanik',
    description='Weld inspection ROS2 package for Gazebo Classic',
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models', 'weld_piece'), glob('models/weld_piece/*')),
        (os.path.join('share', package_name, 'models', 'our_factory_workshop'), glob('models/our_factory_workshop/*.*')),
        (os.path.join('share', package_name, 'models', 'our_factory_workshop', 'meshes'), glob('models/our_factory_workshop/meshes/*')),
    ],
    entry_points={
        'console_scripts': [
            'orchestrator_node = weld_inspection_classic.orchestrator_node:main',
            'weld_inspector_node = weld_inspection_classic.weld_inspector_node:main',
        ],
    },
)