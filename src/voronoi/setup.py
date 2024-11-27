from setuptools import setup
from glob import glob
import os

package_name = 'voronoi'

model_dirs = [
    'tb3_0',
    'tb3_1',
    'tb3_2',
    'tb3_3',
]

model_files = []
for model_dir in model_dirs:
    model_files.extend([
        (os.path.join('share', package_name, 'models', model_dir), 
         glob(os.path.join('models', model_dir, '*.*')))
    ])

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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ] + model_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Akif Bayram',
    maintainer_email='akif@example.com',
    description='Voronoi-based multi-robot exploration using ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voronoi_tb4 = script.voronoi_tb4:main',
            'voronoi_tb3 = script.voronoi_tb3:main',
        ],
    },
)