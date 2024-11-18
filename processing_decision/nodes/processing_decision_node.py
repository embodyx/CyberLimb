#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image as PILImage
import torch
import cv2
from cv_bridge import CvBridge
from interbotix_perception_modules import InterbotixPerceptionModule
import threading
import time

class ProcessingDecisionNode(Node):
    def __init__(self):
        super().__init__('processing_decision_node')
        
        # Subscription to the sensory input
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Publisher for the actions
        self.publisher = self.create_publisher(String, '/openvla_action', 10)
        
        # Subscriber for the instructions
        self.instruction_subscription = self.create_subscription(
            String,
            '/instruction',
            self.instruction_callback,
            10)

        # Load the processor and model
        self.processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
        self.vla = AutoModelForVision2Seq.from_pretrained(
            "openvla/openvla-7b", 
            attn_implementation="flash_attention_2",
            torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True,
            trust_remote_code=True
        ).to("cuda:0")
        
        # Bridge to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()
        
        # Initialize Interbotix Perception Module
        self.perception_module = InterbotixPerceptionModule()
        
        self.instruction = None
        self.image = None
        self.instruction_complete = False

        self.get_logger().info('Processing & Decision Node has been started.')

    def instruction_callback(self, msg):
        self.instruction = msg.data
        self.get_logger().info(f'Received instruction: {self.instruction}')
        self.instruction_complete = False
        self.process_instruction()

    def image_callback(self, msg):
        self.image = msg
        self.get_logger().info('Received image')

    def check_instruction_complete(self, pil_image):
        # Format prompt for checking task completion
        prompt = f"In: {self.instruction} - Is the task complete?\nOut:"

        # Predict task completion
        inputs = self.processor(prompt, pil_image, return_tensors="pt").to("cuda:0", dtype=torch.bfloat16)
        completion_check = self.vla.generate(**inputs, max_length=50)

        completion_text = self.processor.decode(completion_check[0], skip_special_tokens=True)
        self.get_logger().info(f'Task completion check result: {completion_text}')
        
        # Check if the model indicates the task is complete
        positive_responses = ['yes', 'completed', 'finished', 'done']
        negative_responses = ['no', 'not yet', 'incomplete', 'unfinished']

        # Convert to lower case for case-insensitive comparison
        completion_text_lower = completion_text.lower()

        # Check for positive indications of completion
        for response in positive_responses:
            if response in completion_text_lower:
                return True
        
        # Check for negative indications of incomplete task
        for response in negative_responses:
            if response in completion_text_lower:
                return False
        
        # If no clear indication, log and assume incomplete
        self.get_logger().info(f'Uncertain task completion check result: {completion_text}')
        return False

    def process_instruction(self):
        while self.instruction and not self.instruction_complete:
            if self.image:
                # Convert ROS image message to PIL image
                cv_image = self.bridge.imgmsg_to_cv2(self.image, "bgr8")
                pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

                # Use Interbotix perception module to process the image (example)
                detected_objects = self.perception_module.detect_objects(cv_image)

                # Check if the instruction is complete
                if self.check_instruction_complete(pil_image):
                    self.instruction_complete = True
                    self.get_logger().info(f'Instruction completed: {self.instruction}')
                    self.instruction = None
                    break

                # Format prompt for OpenVLA model
                prompt = f"In: What action should the robot take to {self.instruction}?\nOut:"

                # Predict action
                inputs = self.processor(prompt, pil_image, return_tensors="pt").to("cuda:0", dtype=torch.bfloat16)
                action = self.vla.generate(**inputs, max_length=50)

                action_text = self.processor.decode(action[0], skip_special_tokens=True)

                # Publish action
                action_msg = String()
                action_msg.data = action_text
                self.publisher.publish(action_msg)

                self.get_logger().info(f'Published action: {action_msg.data}')

                # Wait for the action to be executed
                time.sleep(2)  # Adjust sleep duration as needed

                # Capture new image and re-evaluate
                self.image = None

def main(args=None):
    rclpy.init(args=args)
    node = ProcessingDecisionNode()

    # Run a thread to take user input
    def take_input():
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

if __name__ == '__main__':
    main()
