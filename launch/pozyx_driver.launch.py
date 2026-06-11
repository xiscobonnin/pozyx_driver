from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pozyx_driver',
            executable='pozyx_driver_node',
            name='pozyx_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'num_anchors': 4,
                'tag_device_id': 'None',
                'algorithm': 1,
                'dimension': 1,
                'height': 1000,
                'frequency': 15,
                'world_frame_id': 'world',
                'tag_frame_id': 'pozyx_tag',
                'do_ranging_attempts': 1,
                #anchors coordinates in mm
                'anchor0_id': '0x673D',
                'anchor0_coordinates': '[1000, 2000, 3000]',
                'anchor1_id': '0x683D',
                'anchor1_coordinates': '[4000, 5000, 6000]',
                'anchor2_id': '0x6A39',
                'anchor2_coordinates': '[1000, 3000, 2000]',
                'anchor3_id': '0x6842',
                'anchor3_coordinates': '[3000, 2000, 1000]',
            }]
        )
    ])
