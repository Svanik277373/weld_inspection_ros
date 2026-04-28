from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'weld_inspection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='svanik',
    description='Weld inspection ROS2 package',

    data_files=[
        # Required for ROS2 package discovery
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),

        # Install worlds + models
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models'), glob('models/**/**/*'),),
    ],

    entry_points={
        'console_scripts': [
            'weld_inspector_node = weld_inspection.weld_inspector_node:main',
            'spawn_welds_node = weld_inspection.spawn_welds_node:main',
        ],
    },
)