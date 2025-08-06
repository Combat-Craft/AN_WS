from setuptools import find_packages, setup
from glob import glob
import os


package_name = 'cam_stream'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tasc',
    maintainer_email='tasc@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_camera_publisher = cam_stream.multi_camera_publisher:main',
            'multi_camera_subscriber = cam_stream.multi_camera_subscriber:main',
            'multi_cam_gps_aruco = cam_stream.multi_cam_gps_aruco:main',
            'gps_node = cam_stream.gps_node:main',
            'distance_tracker = cam_stream.distance_tracker:main'
        ],
    },
)
