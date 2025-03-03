#!/usr/bin/env python3
"""Launch file for starting all cyberlimb nodes in the correct order."""

import os
from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node, SetEnvironmentVariable  # type: ignore


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the cyberlimb nodes."""

    # Get the package directories
    sensory_input_dir = get_package_share_directory("sensory_input")
    processing_decision_dir = get_package_share_directory("processing_decision")
    action_output_dir = get_package_share_directory("action_output")

    # Define paths to parameter files
    sensory_params_file = os.path.join(sensory_input_dir, "config", "params.yml")
    processing_params_file = os.path.join(
        processing_decision_dir, "config", "params.yml"
    )
    action_params_file = os.path.join(action_output_dir, "config", "params.yml")

    # Create a namespace for all nodes
    namespace = 'cyberlimb'

    return LaunchDescription(
        [
            # Set logging level
            SetEnvironmentVariable(
                name='RCUTILS_LOGGING_BUFFERED_STREAM', 
                value='1'
            ),
            Node(
                package="sensory_input",
                executable="sensory_input_node",
                name="sensory_input_node",
                namespace=namespace,
                output="screen",
                parameters=[sensory_params_file],
                emulate_tty=True
            ),
            Node(
                package="processing_decision",
                executable="processing_decision_node",
                name="processing_decision_node",
                namespace=namespace,
                output="screen",
                parameters=[processing_params_file],
                emulate_tty=True
            ),
            Node(
                package="action_output",
                executable="action_output_node",
                name="action_output_node",
                namespace=namespace,
                output="screen",
                parameters=[action_params_file],
                emulate_tty=True
            ),
        ]
    )
