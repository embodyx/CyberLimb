

# CyberLimb: Vision-Language Robot Control

## Project Overview

CyberLimb is a ROS2-based system that enables natural language control of a robotic arm using vision-language AI. The system uses an overhead camera to observe the workspace, processes the visual information with OpenVLA (Vision-Language Assistant), and controls an Interbotix WX250S robotic arm to manipulate objects based on natural language instructions.

### Key Features

- **Natural Language Control**: Control the robot with simple English instructions
- **Vision-Based Reasoning**: Uses OpenVLA to understand the scene and plan actions
- **Real-time Feedback**: Continuously evaluates task completion
- **Modular Architecture**: Separate nodes for sensing, processing, and action

### Hardware Requirements

- Interbotix WX250S robotic arm
- Intel RealSense D415 depth camera
- CUDA-compatible GPU with 16GB+ VRAM (for OpenVLA inference)
- Computer with Ubuntu 22.04 and ROS2 Humble

## System Architecture

The system consists of three main ROS2 nodes:

1. **Sensory Input Node**: Captures images from the RealSense camera and publishes them to ROS2
2. **Processing Decision Node**: Processes images using OpenVLA to generate robot actions
3. **Action Output Node**: Translates high-level actions into robot movements

## Installation

### Prerequisites

```bash
# Install ROS2 Humble (follow instructions at https://docs.ros.org/en/humble/Installation.html)

# Install Interbotix SDK
curl -s https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh | bash -s -- -d humble

# Install RealSense SDK
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# Install Python dependencies
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install transformers pyrealsense2 opencv-python pillow
```

### Building the Project

```bash
# Create a ROS2 workspace
mkdir -p ~/cyberlimb_ws/src
cd ~/cyberlimb_ws/src

# Clone the repository
git clone https://github.com/yourusername/cyberlimb.git .

# Build the workspace
cd ..
colcon build
source install/setup.bash
```

## Configuration

Each node has a configuration file in its respective `config` directory:

- `src/sensory_input/config/params.yml`: Camera configuration
- `src/processing_decision/config/params.yml`: OpenVLA model and topic configuration
- `src/action_output/config/params.yml`: Robot arm configuration

Adjust these files as needed for your specific setup.

## Running the System

### Option 1: Launch All Nodes

```bash
# Source the workspace
source ~/cyberlimb_ws/install/setup.bash

# Launch all nodes
ros2 launch cyberlimb cyberlimb_launch.py
```

### Option 2: Launch Nodes Individually

```bash
# Terminal 1: Launch the sensory input node
ros2 run sensory_input sensory_input_node --ros-args --params-file src/sensory_input/config/params.yml

# Terminal 2: Launch the processing decision node
ros2 run processing_decision processing_decision_node --ros-args --params-file src/processing_decision/config/params.yml

# Terminal 3: Launch the action output node
ros2 run action_output action_output_node --ros-args --params-file src/action_output/config/params.yml
```

## Usage

1. Ensure the camera has a clear view of the workspace
2. Place wooden blocks in the workspace
3. When the processing_decision_node prompts for input, enter a natural language instruction:
   ```
   Enter instruction: Pick up the red block and place it on the blue block
   ```
4. The system will process the instruction, analyze the scene, and control the robot to complete the task
5. When the task is complete, the robot will return to its home position

## Example Instructions

- "Pick up the red block and place it next to the blue block"
- "Stack all blocks in the center of the workspace"
- "Move the isolated block to join the group of blocks"
- "Sort the blocks by color from left to right"

## Troubleshooting

- **Camera not detected**: Ensure the RealSense camera is properly connected and recognized
- **Robot not moving**: Check USB connections and power to the robot
- **CUDA errors**: Verify GPU compatibility and CUDA installation
- **Model loading errors**: Ensure internet connection for first run to download OpenVLA

## Acknowledgments

- OpenVLA team for the vision-language model
- Interbotix for the WX250S robotic arm and SDK
- Intel for the RealSense camera and SDK
