# CyberLimb: A VLA-based Robotic System Implementation

Experiment code for Jake Kemple's UW Master's Thesis:

**"_Evaluating Vision-Language-Action Models in Robotic Manipulation: Performance, Implementation, and Comparison with Deterministic Systems_"**  

📄 [Defense Presentation Slides w/ Video Clips](https://docs.google.com/presentation/d/1ueZkYw0638_mXNqgWYmY4bsO7ufsI6I4i-9ZU_T4mS0/edit?usp=sharing)

📄 [Thesis Paper PDF](https://drive.google.com/file/d/15EkBcMYAEKk2mtwHfVwkD7fEKKC4VoIb/view?usp=drive_link)

---

## Project Overview

CyberLimb is a real-world ROS 2 robotic system designed to compare two approaches to robotic manipulation:

1. A **deterministic perception and task planning pipeline**, based on Interbotix's open-source control tools.
2. A **Vision-Language-Action (VLA) model-based control system**, built on top of the OpenVLA-7B foundation model.

Both systems were tested using a WidowX 250 6-DoF robotic arm, an Intel RealSense D415 camera, and an NVIDIA Jetson AGX Orin for onboard compute and inference. The robot performed pick-and-place tasks under randomized conditions, with each system evaluated using ISO 9283-adapted metrics: accuracy, repeatability, and cycle time.

---

## Repository Structure and Key Components

### 🧠 `openvla/` *(Forked and adapted)*

Fork of the [OpenVLA repository](https://huggingface.co/openvla/openvla-7b), patched and modified to run on Jetson hardware.

- Contains the **main evaluation script** used to run VLA trials:
  - `openvla/experiments/robot/bridge/run_bridgev2_eval.py`
- This script coordinates:
  - RGB image capture
  - Natural language prompt input
  - OpenVLA inference and action decoding
  - Real-time control via Interbotix motion commands

See Section 3.2.3 of the thesis for full architectural details.

---

### 🔁 `bridge_data_robot/` *(Forked and patched for ROS 2)*

Adapted from the official `bridge_data_robot` repository to support:

- Explicit workspace bounds
- End-effector pose initialization
- ROS 2-compatible launch and action publishing
- Stable execution of OpenVLA output in a real robot setup

This serves as the control infrastructure enabling OpenVLA deployment in physical environments.

---

### 💡 `custom/scripts/pick_place.py`

Located at:  
`custom/scripts/pick_place.py`

A deterministic pick-and-place script based on Interbotix's demos, adapted for the experiment. Implements:

- AR tag-based camera-to-robot calibration
- Point cloud segmentation and color-based object detection
- Deterministic Cartesian motion planning
- Rigid object grasp and place routines

Used to generate the baseline results in the thesis.

---

### 🧪 `custom/src/` *(Custom VLA integration – initial attempt)*

A standalone ROS 2-native attempt to integrate OpenVLA directly via three custom nodes:

- `Sensory Input Node`: Captures RGB images from RealSense
- `Processing Decision Node`: Runs OpenVLA inference on images + language input
- `Action Output Node`: Applies action deltas to robot via Interbotix control APIs

Though functional in pipeline structure, this approach failed to complete tasks reliably due to control drift, calibration mismatches, and lack of contextual task awareness. See Section 3.2.2 of the thesis for a complete breakdown.

---

## Acknowledgments

- **OpenVLA team** – for the VLA model and Hugging Face release
- **RAIL & the BridgeData V2 team** – for training datasets and creating robotic integration tools
- **Interbotix** – for the WidowX 250 hardware and open-source SDK
- **Intel** – for the RealSense D415 depth camera
- **NVIDIA** – for the Jetson AGX Orin platform enabling edge inference
