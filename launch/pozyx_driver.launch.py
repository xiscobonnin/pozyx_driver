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
                'anchor0_id': '0x0000',
                'anchor0_coordinates': '[0, 0, 0]',
                'anchor1_id': '0x0000',
                'anchor1_coordinates': '[0, 0, 0]',
                'anchor2_id': '0x0000',
                'anchor2_coordinates': '[0, 0, 0]',
                'anchor3_id': '0x0000',
                'anchor3_coordinates': '[0, 0, 0]',
            }]
        )
    ])
