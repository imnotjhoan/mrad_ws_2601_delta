from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'delta_path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhoan',
    maintainer_email='jhoan.chacon@eia.edu.co',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dijkstra_node = delta_path_planner.delta_path_planner_dijkstra:main',
            'best_first_node = delta_path_planner.delta_path_planner_bestFirst:main',
            'waypoints_node = delta_path_planner.waypoints:main',
        ],
    },
)
