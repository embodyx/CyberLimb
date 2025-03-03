#!/usr/bin/env python3
"""ROS2 node for controlling the WidowX robot arm based on received actions."""
# pylint: disable=assigning-non-slot,no-member

from typing import List, Optional

import re
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from std_msgs.msg import String  # type: ignore
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS  # type: ignore


class ActionOutputNode(Node):
    """Node for executing robot arm actions based on received commands."""

    def __init__(self) -> None:
        """Initialize the node, set up robot arm and subscribers."""
        super().__init__("action_output_node")

        # Declare and get parameters
        self.declare_parameter("action_topic", "/openvla_action")
        
        action_topic = self.get_parameter("action_topic").value

        # Use parameters for subscriptions/publishers
        self.subscription = self.create_subscription(
            String, action_topic, self.execute_action, 10
        )
        self.publisher = self.create_publisher(String, action_topic, 10)

        # Initialize the InterbotixManipulatorXS object for the WidowX arm
        try:
            self.arm = InterbotixManipulatorXS("wx250s", "arm", "gripper")
            self.get_logger().info('Successfully connected to WX250S robot')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot: {str(e)}')
            raise

        # Define home position
        self.home_position = [0.0, 0.0, 0.2]

        # Move to home position at startup
        self.arm.go_to_sleep_pose()  # type: ignore[attr-defined]
        self.get_logger().info("Robot initialized to sleep position")

        self.get_logger().info("Action Output Node has been started.")

    def execute_action(self, msg: String) -> None:
        """Execute the received action command."""
        self.get_logger().info(f"Executing action: {msg.data}")
        action = msg.data.lower()

        try:
            if "move_to_position" in action or "move to position" in action:
                position = self.extract_position(action)
                if position:
                    self.move_to_position(position)
                else:
                    self.get_logger().warn(
                        "Could not extract valid position from action"
                    )

            elif any(
                cmd in action for cmd in ["grip", "close gripper", "grab", "pick"]
            ):
                self.grip()

            elif any(
                cmd in action for cmd in ["release", "open gripper", "let go", "drop"]
            ):
                self.release()

            elif any(cmd in action for cmd in ["return_to_home", "home", "sleep"]):
                self.return_to_home()

            else:
                self.get_logger().warn(f"Unknown action: {action}")

        except (RuntimeError, ValueError) as e:
            self.get_logger().error(f"Error executing action: {str(e)}")

    def extract_position(self, action: str) -> Optional[List[float]]:
        """Extract position coordinates from action text."""
        try:
            coords = re.findall(r"[-+]?\d*\.\d+|\d+", action)

            if len(coords) >= 3:
                x = float(coords[0])
                y = float(coords[1])
                z = float(coords[2])

                # Apply safety limits
                x = max(-0.5, min(0.5, x))
                y = max(-0.5, min(0.5, y))
                z = max(0.05, min(0.4, z))

                return [x, y, z]

            self.get_logger().warn(f"Could not find 3 coordinates in: {action}")
            return None

        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error extracting position: {str(e)}")
            return None

    def move_to_position(self, position: List[float]) -> None:
        """Move the robot arm to the specified position."""
        self.get_logger().info(f"Moving to position {position}")
        try:
            self.arm.set_ee_pose_components(  # type: ignore[attr-defined]
                x=position[0], y=position[1], z=position[2], moving_time=2.0
            )
            self.get_logger().info(f"Moved to position {position}")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to move to position: {str(e)}")

    def grip(self) -> None:
        """Close the gripper."""
        try:
            self.arm.gripper.grasp(2.0)  # type: ignore[attr-defined]
            self.get_logger().info("Gripper closed")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to grip: {str(e)}")

    def release(self) -> None:
        """Open the gripper."""
        try:
            self.arm.gripper.release(2.0)  # type: ignore[attr-defined]
            self.get_logger().info("Gripper opened")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to release: {str(e)}")

    def return_to_home(self) -> None:
        """Return the robot to home/sleep position."""
        try:
            self.get_logger().info("Returning to home position")
            self.arm.go_to_sleep_pose()  # type: ignore[attr-defined]
            self.get_logger().info("Robot returned to home position")
        except RuntimeError as e:
            self.get_logger().error(f"Failed to return to home: {str(e)}")


def main(args: Optional[str] = None) -> None:
    """Initialize and run the action output node."""
    rclpy.init(args=args)
    node = ActionOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
