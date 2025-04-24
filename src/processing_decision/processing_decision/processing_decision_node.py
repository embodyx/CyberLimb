#!/usr/bin/env python3
"""ROS 2 node that listens to /camera/image_raw and feeds RGB frames to an OpenVLA model."""

import json
import threading
import time
from typing import Optional

import numpy as np
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image as PILImage
from cv_bridge import CvBridge


class ProcessingDecisionNode(Node):
    """Consumes camera frames + a goal instruction and publishes OpenVLA actions."""

    def __init__(self) -> None:
        super().__init__("processing_decision_node")

        # ------------ Parameters ------------
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("action_topic", "/openvla_action")
        self.declare_parameter("instruction_topic", "/instruction")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("inference_interval", 1.0)

        camera_topic      = self.get_parameter("camera_topic").value
        action_topic      = self.get_parameter("action_topic").value
        instruction_topic = self.get_parameter("instruction_topic").value
        self.device       = self.get_parameter("device").value
        self.interval     = self.get_parameter("inference_interval").value

        # ------------ ROS I/O ------------
        self.create_subscription(Image,  camera_topic,      self._image_cb,       5)
        self.create_subscription(String, instruction_topic, self._instruction_cb, 5)
        self.action_pub = self.create_publisher(String, action_topic, 10)

        # ------------ Model ------------
        self.processor = AutoProcessor.from_pretrained(
            "openvla/openvla-7b", trust_remote_code=True
        )
        self.vla = AutoModelForVision2Seq.from_pretrained(
            "openvla/openvla-7b",
            torch_dtype=torch.bfloat16,
            low_cpu_mem_usage=True,
            trust_remote_code=True,
            attn_implementation="sdpa",
        ).to(self.device)

        # ------------ State ------------
        self.bridge         = CvBridge()
        self.latest_image   = None     # type: Optional[Image]
        self.instruction    = None     # type: Optional[str]
        self.processing     = False    # gate so only one instruction runs at a time
        self.get_logger().info("ProcessingDecisionNode ready.")

    # ----------------- ROS Callbacks -----------------
    def _image_cb(self, msg: Image) -> None:
        """Store latest frame (RGB encoding is guaranteed by upstream node)."""
        self.latest_image = msg

    def _instruction_cb(self, msg: String) -> None:
        """Start a background thread to fulfil a new instruction."""
        if self.processing:
            self.get_logger().warn("Ignoring new instruction: already busy.")
            return
        self.instruction  = msg.data.strip()
        self.processing   = True
        threading.Thread(target=self._work_loop, daemon=True).start()

    # ----------------- Worker -----------------
    def _work_loop(self) -> None:
        while self.processing and self.instruction:
            if self.latest_image is None:
                time.sleep(0.05)
                continue

            # ---- Convert ROS Image → NumPy (RGB) → PIL (RGB) ----
            rgb_np = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="rgb8")
            pil_im = PILImage.fromarray(rgb_np)

            # ---- Run model ----
            prompt = f"In: What action should the robot take to {self.instruction}\nOut:"
            inputs = self.processor(prompt, pil_im, return_tensors="pt").to(
                device=self.device, dtype=torch.bfloat16
            )
            action = self.vla.predict_action(**inputs,
                                             unnorm_key="bridge_orig",
                                             do_sample=False)

            # ---- Publish ----
            out = String()
            out.data = json.dumps(action.tolist())
            self.action_pub.publish(out)
            self.get_logger().info(f"Action published: {out.data}")

            # ---- Wait for next frame ----
            self.latest_image = None
            time.sleep(self.interval)

        self.processing  = False
        self.instruction = None

# ----------------- main -----------------
def main(args: Optional[str] = None) -> None:
    rclpy.init(args=args)
    node = ProcessingDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
