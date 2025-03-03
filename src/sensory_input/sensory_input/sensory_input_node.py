#!/usr/bin/env python3
"""ROS2 node for capturing and publishing images from a RealSense camera."""
# pylint: disable=assigning-non-slot,no-member

from typing import Optional

import numpy as np
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import Image  # type: ignore
from cv_bridge import CvBridge  # type: ignore
import pyrealsense2 as rs  # type: ignore[import, attr-defined]

class SensoryInputNode(Node):
    """Node for handling RealSense camera input and publishing ROS images."""

    def __init__(self) -> None:
        """Initialize the node, configure camera streams and set up publishers."""
        super().__init__('sensory_input_node')

        # Declare parameters
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('publish_rate', 10.0)

        # Get parameters
        width = self.get_parameter('camera_width').value
        height = self.get_parameter('camera_height').value
        fps = self.get_parameter('camera_fps').value
        rate = self.get_parameter('publish_rate').value

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0/rate, self.timer_callback)
        self.bridge = CvBridge()

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Use parameters for camera configuration
        self.config.enable_stream(
            rs.stream.color,
            width,
            height,
            rs.format.bgr8,
            fps
        )

        # Start streaming
        try:
            self.pipeline.start(self.config)
            self.get_logger().info('RealSense camera initialized successfully')
        except RuntimeError as e:  # RealSense specific exception
            self.get_logger().error(f'Failed to start RealSense camera: {str(e)}')
            raise

        self.get_logger().info('Sensory Input Node has been started.')

    def timer_callback(self) -> None:
        """Capture frames from RealSense camera and publish as ROS Image messages."""
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                self.get_logger().warn('No color frame received')
                return

            # Convert image to numpy array
            color_image = np.asanyarray(color_frame.get_data())

            # Convert numpy array to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")  # type: ignore

            # Add timestamp
            ros_image.header.stamp = self.get_clock().now().to_msg()  # type: ignore
            ros_image.header.frame_id = "camera_link"  # type: ignore

            self.publisher.publish(ros_image)
        except (RuntimeError, ValueError) as e:  # More specific exceptions
            self.get_logger().error(f'Error in timer_callback: {str(e)}')

    def __del__(self) -> None:
        """Clean up resources by stopping the RealSense pipeline."""
        # Ensure pipeline is properly stopped when node is destroyed
        try:
            self.pipeline.stop()
            self.get_logger().info('RealSense pipeline stopped')
        except RuntimeError:  # Removed unused 'e' variable
            pass

def main(args: Optional[str] = None) -> None:
    """Initialize and run the sensory input node."""
    rclpy.init(args=args)
    node = SensoryInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
