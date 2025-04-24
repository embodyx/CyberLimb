#!/usr/bin/env python3
"""ROS 2 node – drive WidowX-250S from OpenVLA 7-D action vectors.

▪ Uses tf2 if a camera→base_link transform is on the tree.  
▪ Otherwise loads the static rotation from static_transforms.yaml.  
▪ Properly inverts that rotation (camera → base) before applying deltas.
"""

import json, os, time, yaml
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from tf2_ros import (Buffer, TransformListener, LookupException,
                     TimeoutException, TransformException)
import tf_transformations as tft
from ament_index_python.packages import get_package_share_directory


class ActionOutputNode(Node):
    CAM_FRAME  = "camera_color_optical_frame"
    BASE_FRAME = "wx250s/base_link"

    # ───────────────────────────────────────────────────────────────
    def __init__(self) -> None:
        super().__init__("action_output_node")

        # parameters
        self.declare_parameter("action_topic", "/openvla_action")
        self.declare_parameter("robot_name",   "wx250s")
        topic      = self.get_parameter("action_topic").value
        robot_name = self.get_parameter("robot_name").value

        # tf2
        self.tf_buf = Buffer(rclpy.duration.Duration(seconds=5))
        self.tf_lis = TransformListener(self.tf_buf, self, spin_thread=True)

        # YAML fallback
        self.R_yaml = self._load_yaml_rotation()

        # arm driver (positional args: model, robot_name)
        self.arm = InterbotixManipulatorXS(
            robot_model="wx250s",
            robot_name=robot_name,
            group_name="arm",
            gripper_name="gripper"
        )

        self.arm.arm.go_to_home_pose()
        self.gripper_closed = False
        self.get_logger().info("Arm ready.")

        # subscriber
        self.create_subscription(String, topic, self.execute_action, 10)

    # ───────────────────────────────────────────────────────────────
    def _load_yaml_rotation(self) -> np.ndarray:
        """Return 3×3 rotation matrix R_cam→base from static_transforms.yaml."""
        pkg  = get_package_share_directory("interbotix_xsarm_perception")
        path = os.path.join(pkg, "config", "static_transforms.yaml")
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        transforms = data["transforms"] if isinstance(data, dict) else data
        for tr in transforms:
            if tr["frame_id"] == self.CAM_FRAME and tr["child_frame_id"] == self.BASE_FRAME:
                q = [tr["qx"], tr["qy"], tr["qz"], tr["qw"]]
                return R.from_quat(q).as_matrix().T   # transpose = invert
        raise RuntimeError("camera→base rotation not found in YAML")

    # ───────────────────────────────────────────────────────────────
    def cam_delta_to_base(self, dx: float, dy: float, dz: float) -> np.ndarray:
        """Rotate (dx,dy,dz) from camera to base frame; fall back to YAML."""
        try:
            tr = self.tf_buf.lookup_transform(
                self.BASE_FRAME, self.CAM_FRAME, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2))
            T    = tft.compose_matrix(
                      translate=(tr.transform.translation.x,
                                 tr.transform.translation.y,
                                 tr.transform.translation.z),
                      angles=tft.euler_from_quaternion(
                                 [tr.transform.rotation.x,
                                  tr.transform.rotation.y,
                                  tr.transform.rotation.z,
                                  tr.transform.rotation.w]))
            R_cb = T[:3, :3].T        # invert rotation: camera→base
        except TransformException:
            R_cb = self.R_yaml         # static fallback

        return R_cb @ np.array([dx, dy, dz])

    # ───────────────────────────────────────────────────────────────
    def execute_action(self, msg: String) -> None:
        try:
            vec = json.loads(msg.data)
            if not (isinstance(vec, list) and len(vec) == 7):
                return
            dx, dy, dz, droll, dpitch, dyaw, g = vec
            if all(abs(v) < 1.5e-3 for v in vec[:-1]):
                return

            time.sleep(0.5)                          # settle previous move
            ee_T = self.arm.arm.get_ee_pose()
            pos  = ee_T[:3, 3]
            roll, pitch, yaw = R.from_matrix(ee_T[:3, :3]).as_euler("xyz")

            dpos = self.cam_delta_to_base(dx, dy, dz)

            tgt = dict(
                x=pos[0] + dpos[0],
                y=pos[1] + dpos[1],
                z=pos[2] + dpos[2],
                roll=roll  + droll,
                pitch=pitch + dpitch,
                yaw=yaw   + dyaw,
            )
            self.arm.arm.set_ee_pose_components(**tgt)

            # gripper
            if g >= 0.5 and not self.gripper_closed:
                self.arm.gripper.grasp(1.0); self.gripper_closed = True
            elif g < 0.5 and self.gripper_closed:
                self.arm.gripper.release(0.0); self.gripper_closed = False

        except Exception as e:                       # broad catch for runtime
            self.get_logger().error(f"Execution error: {e}")

# ------------------------------------------------------------------
def main(args: Optional[str] = None) -> None:
    rclpy.init(args=args)
    node = ActionOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
