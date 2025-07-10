from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'navigation'

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
    maintainer='bao',
    maintainer_email='bao@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = navigation.gps_node:main',
            'gps_display = navigation.gps_display:main',
            'camera_publisher = navigation.camera_publisher:main',
            'multi_camera_publisher = navigation.multi_camera_publisher:main',
            'aruco_marker_publisher = navigation.aruco_marker_publisher:main',
            'combined_gps_aruco = navigation.combined_gps_aruco_node:main',
            'camera_subscriber = navigation.camera_subscriber:main',
            'multi_camera_publisher_gst = navigation.multi_camera_publisher_gst:main',
            'multi_camera_subscriber_gst = navigation.multi_camera_subscriber_gst:main',
            'gst_pub = navigation.gst_pub:main',
        ],
    },
)
