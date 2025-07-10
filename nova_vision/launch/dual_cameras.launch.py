from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nova_vision',
            executable='multi_camera_publisher',
            name='camera0_node',
            arguments=['0'] #The media stream for the camera (eg. 0, -1 or '/dev/video2')
        ),
        Node(
            package='nova_vision',
            executable='multi_camera_publisher',
            name='camera1_node',
            arguments=['1'] #The media stream for the camera (eg. '/dev/video2')
        ),
        Node(
            package='nova_vision', 
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
