#!/usr/bin/env python3
"""Launch file for starting all cyberlimb nodes in the correct order."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for the cyberlimb nodes."""

    # Get the package directories
    sensory_input_dir = get_package_share_directory("sensory_input")
    processing_decision_dir = get_package_share_directory("processing_decision")
    action_output_dir = get_package_share_directory("action_output")
    interbotix_dir = get_package_share_directory("interbotix_xsarm_control")

    # Define paths to parameter files
    sensory_params_file = os.path.join(sensory_input_dir, "config", "params.yml")
    processing_params_file = os.path.join(processing_decision_dir, "config", "params.yml")
    action_params_file = os.path.join(action_output_dir, "config", "params.yml")

    # Namespace for all nodes
    namespace = 'cyberlimb'

    # Check parameter files
    sensory_params = [sensory_params_file] if os.path.exists(sensory_params_file) else []
    if not sensory_params:
        print(f"Warning: Parameter file not found: {sensory_params_file}")
    processing_params = [processing_params_file] if os.path.exists(processing_params_file) else []
    if not processing_params:
        print(f"Warning: Parameter file not found: {processing_params_file}")
    action_params = [action_params_file] if os.path.exists(action_params_file) else []
    if not action_params:
        print(f"Warning: Parameter file not found: {action_params_file}")

    # Interbotix XS Arm launch
    interbotix_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(interbotix_dir, 'launch', 'xsarm_control.launch.py')
        ),
        launch_arguments={
            'robot_model': 'wx250s',
            'use_sim': 'false',
        }.items()
    )

    # Our nodes under namespace
    cyberlimb_group = GroupAction([
        PushRosNamespace(namespace),
        Node(
            package="sensory_input",
            executable="sensory_input_node",
            name="sensory_input_node",
            output="screen",
            parameters=sensory_params,
            emulate_tty=True,
            remappings=[('/camera/image_raw', 'camera/image_raw')]
        ),
        Node(
            package="processing_decision",
            executable="processing_decision_node",
            name="processing_decision_node",
            output="screen",
            parameters=processing_params,
            emulate_tty=True,
            remappings=[
                ('/camera/image_raw', 'camera/image_raw'),
                ('/instruction', 'instruction'),
                ('/openvla_action', 'openvla_action'),
            ]
        ),
        Node(
            package="action_output",
            executable="action_output_node",
            name="action_output_node",
            output="screen",
            parameters=[*action_params, {'robot_name':'wx250s'}],
            emulate_tty=True,
            remappings=[('/openvla_action', 'openvla_action')]
        ),
    ])

    return LaunchDescription([
        # Ensure logs flush promptly
        SetEnvironmentVariable(
            name='RCUTILS_LOGGING_BUFFERED_STREAM',
            value='1'
        ),
        # Start interbotix arm driver
        interbotix_launch,
        # Delay our nodes to ensure driver is ready
        TimerAction(
            period=3.0,
            actions=[cyberlimb_group]
        ),
    ])
