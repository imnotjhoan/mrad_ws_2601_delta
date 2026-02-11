from setuptools import find_packages, setup

package_name = 'delta_wall_following'

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
            'delta_wall_following_node = delta_wall_following.dist_finder:main',
            'control_node = delta_wall_following.control:main',
        ],
    },
)
