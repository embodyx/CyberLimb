from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='processing_decision',
            executable='processing_decision_node.py',
            name='processing_decision_node',
            output='screen',
            parameters=[
                # Load configuration parameters
                {'config_file_path': 'config/params.yaml'}
            ]
        )
    ])
