#!/usr/bin/env python3
"""ROS 2 node that captures RealSense RGB frames and publishes them unchanged."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import threading


class SensoryInputNode(Node):
    def __init__(self) -> None:
        super().__init__('sensory_input_node')

        # ---------- User-tunable parameters ----------
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('publish_rate', 5.0)

        width  = self.get_parameter('camera_width').value
        height = self.get_parameter('camera_height').value
        fps    = self.get_parameter('camera_fps').value
        rate   = self.get_parameter('publish_rate').value

        self.get_logger().info(f'Publishing at {rate:.2f} Hz')

        # ---------- ROS 2 publisher ----------
        self.bridge    = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # ---------- RealSense pipeline ----------
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        # Force camera to give RGB (not BGR) so no later swap is needed
        cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)

        try:
            profile = self.pipeline.start(cfg)
            sp = profile.get_stream(rs.stream.color).as_video_stream_profile()
            self.get_logger().info(
                f'RealSense color stream {sp.width}×{sp.height}@{sp.fps} rgb8'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start RealSense camera: {e}')
            raise

        # ---------- Capture thread ----------
        self.latest_frame = None
        self.running      = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        # ---------- Timer for publishing ----------
        self.timer = self.create_timer(1.0 / rate, self._publish_latest)

    # ------------ Worker methods ------------
    def _capture_loop(self) -> None:
        while self.running:
            try:
                frames       = self.pipeline.wait_for_frames()
                color_frame  = frames.get_color_frame()
                if color_frame:
                    # Already RGB thanks to rgb8 stream
                    self.latest_frame = np.asarray(color_frame.get_data())
            except Exception as e:
                self.get_logger().error(f'Frame capture error: {e}')

    def _publish_latest(self) -> None:
        frame = self.latest_frame
        if frame is None:
            return
        try:
            ros_msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            ros_msg.header.stamp    = self.get_clock().now().to_msg()
            ros_msg.header.frame_id = 'camera_link'
            self.publisher.publish(ros_msg)
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')

    # ------------ Clean shutdown ------------
    def destroy_node(self) -> None:
        self.running = False
        if self.capture_thread.is_alive():
            self.capture_thread.join()
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensoryInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
