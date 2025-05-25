#!/usr/bin/env python3
"""Drive a WidowX-250 S from OpenVLA 7-DoF action vectors.

* Camera-frame → base-frame conversion via TF2 (YAML fallback)
* Live-tunable gains:
      translation_scale  [m / +1.0]  (default 0.10)
      rotation_scale     [rad / +1.0] (default 0.0 – start w/ pure position)
"""

import json, os, yaml
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

from tf2_ros import Buffer, TransformListener, TransformException
from ament_index_python.packages import get_package_share_directory

import threading
from rclpy.executors import MultiThreadedExecutor

class ActionOutputNode(Node):
    def __init__(self) -> None:
        super().__init__("action_output_node")

        # -------- parameters --------
        self.declare_parameter("action_topic",      "/openvla_action")
        self.declare_parameter("robot_name",        "wx250s")

        topic      = self.get_parameter("action_topic").value
        robot_name = self.get_parameter("robot_name").value

        prefix              = f"{robot_name}/" if robot_name else ""
        self.BASE_FRAME     = prefix + "base_link"
        self.SOURCE_FRAME   = prefix + "ee_gripper_link"

        # -------- Arm driver --------
        self.arm = InterbotixManipulatorXS(
            robot_model="wx250s",
            robot_name=robot_name,
            group_name="arm",
            gripper_name="gripper",
        )

        # self.arm.arm.go_to_home_pose()
        self.arm.arm.err_tol_ee_pos = 1e-3    # tighten position tol
        self.arm.arm.err_tol_ee_ori = 1e-4    # tighten orientation tol
        self.arm.arm.set_ee_pose_components(x=0.25, z=0.2)
        self.gripper_closed = False
        self.get_logger().info("ActionOutputNode ready.")

        # -------- Subscriber --------
        self.create_subscription(String, topic, self.execute_action, 10)

    def execute_action(self, msg: String) -> None:
        try:
            v = json.loads(msg.data)
            if not (isinstance(v, list) and len(v) == 7):
                return

            dx, dy, dz, droll, dpitch, dyaw, g = np.array(v, dtype=float)

            # if np.allclose([dx, dy, dz, droll, dpitch, dyaw], 0, atol=1e-4):
            #     return

            if not hasattr(self, "tf_buffer"):
                # first time through: create TF2 buffer/listener once per node
                self.tf_buffer   = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

            try:
                t = self.tf_buffer.lookup_transform(
                        target_frame=self.BASE_FRAME,
                        source_frame=self.SOURCE_FRAME,
                        time=rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(
                                        seconds=0.25))
            except TransformException as exc:
                self.get_logger().warn(f"TF lookup failed: {exc}")
                return

            p_w = np.array([t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z])

            R_w = R.from_quat([ t.transform.rotation.x,
                                t.transform.rotation.y,
                                t.transform.rotation.z,
                                t.transform.rotation.w ]).as_matrix()

            p_new = p_w + np.array([dx, dy, dz])
            # self.get_logger().info(f"Current position: {p_w}, New position: {p_new}")

            R_delta = R.from_euler('xyz', [droll, dpitch, dyaw]).as_matrix()
            R_new   = R_delta @ R_w
            rpy_new = R.from_matrix(R_new).as_euler('xyz')

            self.get_logger().info(f"p_new[0]: {p_new[0]}, p_new[1]: {p_new[1]}, p_new[2]: {p_new[2]}")
            self.get_logger().info(f"rpy_new[0]: {rpy_new[0]}, rpy_new[1]: {rpy_new[1]}, rpy_new[2]: {rpy_new[2]}")

            arm_jpos = [ self.arm.core.joint_states.position[self.arm.core.js_index_map[n]]
                        for n in self.arm.arm.group_info.joint_names ]
            self.arm.arm.joint_commands = arm_jpos

            # 1) run IK only – no motion yet
            self.arm.arm.set_ee_pose_components(
                x=p_new[0], y=p_new[1], z=p_new[2],
                roll=rpy_new[0], pitch=rpy_new[1], yaw=rpy_new[2],
                blocking=False)                      # ← no command sent

            cmd  = self.arm.arm.joint_commands       # IK result
            prev = arm_jpos                          # posture we started from

            # 2) clip the roll joints
            band = 0.17            # ±10° per iteration
            for idx in (4, 5):     # 4 = forearm_roll, 5 = wrist_rotate
                delta = cmd[idx] - prev[idx]
                if abs(delta) > band:
                    cmd[idx] = prev[idx] + np.sign(delta) * band

            # 3) now publish the *clipped* command
            self.arm.arm.set_joint_positions(cmd, blocking=False)

            # gripper
            if g >= 0.5 and not self.gripper_closed:
                self.arm.gripper.grasp(1.0);  self.gripper_closed = True
            elif g < 0.5 and self.gripper_closed:
                self.arm.gripper.release(0.0); self.gripper_closed = False

        except Exception as e:  # keep node alive
            self.get_logger().error(f"Execution error: {e}")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActionOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
