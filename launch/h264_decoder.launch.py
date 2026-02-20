from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/cam0/h264'),
        DeclareLaunchArgument('output_topic', default_value='/cam0/image_raw'),
        DeclareLaunchArgument('frame_id', default_value='cam0'),
        Node(
            package='h264_decoder_node',
            executable='h264_decoder_node',
            name='h264_decoder_node',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
            output='screen',
        ),
    ])
