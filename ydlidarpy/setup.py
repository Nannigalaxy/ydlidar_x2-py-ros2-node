from setuptools import find_packages, setup

package_name = 'ydlidarpy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Nandan Manjunatha',
    maintainer_email='nannigalaxy25@gmail.com',
    description='ROS2 node for YDLidar X2 LiDAR sensor',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ydlidar_x2_node = ydlidarpy.x2_node:main',
        ],
    },
)
