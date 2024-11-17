from setuptools import setup
from glob import glob
import os

package_name = 'voronoi'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'script.voronoi_tb4',
        'script.voronoi_tb3'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files in the launch directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'models'), glob('models/*/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Akif Bayram',
    maintainer_email='akif@example.com',
    description='Voronoi-based multi-robot exploration using ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dijkstra = script.dijkstra:main', 
            'voronoi_tb4 = script.voronoi_tb4:main',
            'voronoi_tb3 = script.voronoi_tb3:main',
        ],
    },
)
