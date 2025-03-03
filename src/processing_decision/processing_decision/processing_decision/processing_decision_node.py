#!/usr/bin/env python3
"""ROS2 node for processing images and making decisions using OpenVLA model."""
# pylint: disable=assigning-non-slot,no-member

from typing import Optional
import threading
import time

import cv2  # type: ignore
import torch
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import Image  # type: ignore
from std_msgs.msg import String  # type: ignore
from transformers import AutoModelForVision2Seq, AutoProcessor  # type: ignore
from PIL import Image as PILImage  # type: ignore
from cv_bridge import CvBridge  # type: ignore


class ProcessingDecisionNode(Node):
    """Node for processing camera input and making decisions using AI model."""

    def __init__(self) -> None:
        """Initialize the node, set up subscribers, publishers and AI model."""
        super().__init__("processing_decision_node")

        # Declare and get parameters
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("action_topic", "/openvla_action")
        self.declare_parameter("instruction_topic", "/instruction")
        self.declare_parameter("device", "cuda:0")

        camera_topic = self.get_parameter("camera_topic").value
        action_topic = self.get_parameter("action_topic").value
        instruction_topic = self.get_parameter("instruction_topic").value
        self.device = self.get_parameter("device").value

        # Subscription to the sensory input
        self.subscription = self.create_subscription(
            Image, camera_topic, self.image_callback, 10
        )

        # Publisher for the actions
        self.publisher = self.create_publisher(String, action_topic, 10)

        # Subscriber for the instructions
        self.instruction_subscription = self.create_subscription(
            String, instruction_topic, self.instruction_callback, 10
        )

        # Load the processor
        self.processor = AutoProcessor.from_pretrained(
            "openvla/openvla-7b", trust_remote_code=True
        )

        # Initialize model with Flash Attention 2.0 on GPU
        self.vla = AutoModelForVision2Seq.from_pretrained(
            "openvla/openvla-7b",
            attn_implementation="flash_attention_2",
            torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True,
            trust_remote_code=True,
        )

        # Move the model to GPU
        self.vla.to(self.device)

        # Bridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        self.instruction: Optional[str] = None
        self.image: Optional[Image] = None
        self.instruction_complete = False

        self.get_logger().info("Processing & Decision Node has been started.")

    def instruction_callback(self, msg: String) -> None:
        """Handle incoming instruction messages."""
        self.instruction = msg.data
        self.get_logger().info(f"Received instruction: {self.instruction}")
        self.instruction_complete = False
        self.process_instruction()

    def image_callback(self, msg: Image) -> None:
        """Handle incoming image messages."""
        self.image = msg
        self.get_logger().info("Received image")

    def check_instruction_complete(self, pil_image: PILImage) -> bool:
        """Check if the current instruction has been completed."""
        if not self.instruction:
            return False

        prompt = f"In: {self.instruction} - Is the task complete?\nOut:"

        inputs = self.processor(prompt, pil_image, return_tensors="pt").to(
            self.device, dtype=torch.bfloat16
        )

        completion_check = self.vla.generate(**inputs, max_length=50)
        completion_text = self.processor.decode(
            completion_check[0], skip_special_tokens=True
        )

        self.get_logger().info(f"Task completion check result: {completion_text}")

        positive_responses = ["yes", "completed", "finished", "done"]
        negative_responses = ["no", "not yet", "incomplete", "unfinished"]

        completion_text_lower = completion_text.lower()

        if any(response in completion_text_lower for response in positive_responses):
            return True

        if any(response in completion_text_lower for response in negative_responses):
            return False

        self.get_logger().info(
            f"Uncertain task completion check result: {completion_text}"
        )
        return False

    def process_instruction(self) -> None:
        """Process the current instruction using the AI model."""
        while self.instruction and not self.instruction_complete and self.image:
            # Convert ROS image message to PIL image
            cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
            pil_image = PILImage.fromarray(
                cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  # type: ignore[attr-defined]
            )

            if self.check_instruction_complete(pil_image):
                self.instruction_complete = True
                self.get_logger().info(f"Instruction completed: {self.instruction}")

                action_msg = String()
                action_msg.data = "return_to_home"
                self.publisher.publish(action_msg)
                self.get_logger().info(
                    "Task complete - returning robot to home position"
                )

                self.instruction = None
                break

            prompt = (
                "In: I have a robot arm with a gripper that can pick up and move "
                "small wooden blocks. The camera is mounted above the workspace "
                f"looking down. Based on the image, what specific action should "
                f"the robot take to {self.instruction}? Provide a single, clear "
                f"action like 'move_to_position x y z', 'grip', or 'release'.\nOut:"
            )

            inputs = self.processor(prompt, pil_image, return_tensors="pt").to(
                self.device, dtype=torch.bfloat16
            )

            action = self.vla.generate(**inputs, max_length=100)
            action_text = self.processor.decode(action[0], skip_special_tokens=True)

            action_msg = String()
            action_msg.data = action_text
            self.publisher.publish(action_msg)

            self.get_logger().info(f"Published action: {action_msg.data}")

            time.sleep(3)
            self.image = None


def main(args: Optional[str] = None) -> None:
    """Initialize and run the processing decision node."""
    rclpy.init(args=args)
    node = ProcessingDecisionNode()

    def take_input() -> None:
        """Thread function to handle user input."""
        while rclpy.ok():
            instruction = input("Enter instruction: ")
            instruction_msg = String()
            instruction_msg.data = instruction
            node.instruction_callback(instruction_msg)

    input_thread = threading.Thread(target=take_input, daemon=True)
    input_thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
