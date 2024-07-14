#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class SensoryInputNode(Node):
    def __init__(self):
        super().__init__('sensory_input_node')

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10Hz
        self.bridge = CvBridge()

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)

        self.get_logger().info('Sensory Input Node has been started.')

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        # Convert image to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert numpy array to ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.publisher.publish(ros_image)
        self.get_logger().info('Published an image')

def main(args=None):
    rclpy.init(args=args)
    node = SensoryInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
