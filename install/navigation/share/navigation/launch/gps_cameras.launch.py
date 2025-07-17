from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation', # Replace with your GPS driver package name
            executable='gps_node', # Replace with your GPS driver executable
            name='gps_node',
            output='screen',
            parameters=[
                # Add any GPS specific parameters here (e.g., port, baud_rate)
                {'port': '/dev/ttyACM0'},
                {'baud_rate': 9600},
            ]
        ),
        Node(
            package='navigation',
            executable='camera_publisher',
            name='camera0_node',
            arguments=['-1'] #The media stream for the camera (eg. 0, -1 or '/dev/video2')
        ),
        # Node(
        #     package='navigation',
        #     executable='multi_camera_publisher',
        #     name='camera1_node',
        #     arguments=['1'] #The media stream for the camera (eg. '/dev/video2')
        # ),
        Node(
            package='navigation', 
            executable='gps_display',
            name='gps_display',
            output='screen',
            # You can remap topics here if your script uses different topic names
            # remappings=[
            #     ('/image_raw', '/camera/image_raw'),
            #     ('/gps/fix', '/your_gps_topic'),
            # ]
        ),
    ])
