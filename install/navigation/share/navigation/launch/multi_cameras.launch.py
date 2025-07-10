from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='gps_node',
            name='gps_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baud_rate': 9600},
            ]
        ),
        Node(
            package='navigation',
            executable='combined_gps_aruco',
            name='combined_gps_aruco_node',
            output='screen',
            arguments=['0'],  # camera device ID if needed
        ),
        Node(
            package='navigation',
            executable='multi_camera_publisher',
            name='camera0_node',
            arguments=['2', '0']  # Pass both camera devices to one multi_camera_publisher node
        ),
    ])

