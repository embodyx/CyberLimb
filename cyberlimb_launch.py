from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensory_input',
            executable='sensory_input_node.py',
            name='sensory_input_node',
            output='screen'
        ),
        Node(
            package='processing_decision',
            executable='processing_decision_node.py',
            name='processing_decision_node',
            output='screen'
        ),
        Node(
            package='action_output',
            executable='action_output_node.py',
            name='action_output_node',
            output='screen'
        ),
    ])
