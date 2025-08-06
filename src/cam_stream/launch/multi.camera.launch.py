from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cam_stream',
            executable='multi_camera_publisher',
            name='multi_camera_publisher',
            output='screen',
            
        ),
        Node(
            package='cam_stream',
            executable='gps_node',
            name='gps_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyACM0'},
                {'baud_rate': 57600},
            ]
        ),
        Node(
            package='cam_stream',
            executable='multi_cam_gps_aruco',
            name='multi_cam_gps_aruco',
            output='screen',
        ),
        Node(
            package='cam_stream',
            executable='distance_tracker',
            name='distance_tracker',
            output='screen',
            
        )
    ])

