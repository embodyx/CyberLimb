#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ActionOutputNode(Node):
    def __init__(self):
        super().__init__('action_output_node')
        self.subscription = self.create_subscription(
            String,
            '/openvla_action',
            self.execute_action,
            10)
        self.get_logger().info('Action Output Node has been started.')

    def execute_action(self, msg):
        self.get_logger().info(f'Executing action: {msg.data}')
        # Placeholder for actual actuation logic
        # Perform the action here

def main(args=None):
    rclpy.init(args=args)
    node = ActionOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
