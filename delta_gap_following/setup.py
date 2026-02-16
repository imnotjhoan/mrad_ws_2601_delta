from setuptools import find_packages, setup

package_name = 'delta_gap_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'control_gap_node=delta_gap_following.control_gap_node:main',
            'gap_distance_node=delta_gap_following.gap_distance_node:main',
        ],
    },
)
