from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'nova_vision'

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
    maintainer='toni',
    maintainer_email='tonithetutor@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_subscriber = nova_vision.camera_subscriber:main',
            # Uncomment these if you want to use them
            #'line_follower = nova_vision.line_follower:main',
            #'advanced_line_follower = nova_vision.advanced_line_follower:main',
            'aruco_detect = nova_vision.aruco_detect:main',
            'object_tracker = nova_vision.object_tracker:main',
            'camera_publisher = nova_vision.camera_publisher:main',
            'multi_camera_publisher = nova_vision.multi_camera_publisher:main',
            'gps_display = nova_vision.gps_display:main',
            'gps_aruco = nova_vision.gps_aruco:main',
            'gps_node = nova_vision.gps_node:main',
        ],
    },
)
