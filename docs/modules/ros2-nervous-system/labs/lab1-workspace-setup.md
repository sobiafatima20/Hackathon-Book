# Lab 1: ROS 2 Workspace Setup

## Overview

In this lab, you will set up a ROS 2 Humble Hawksbill workspace on Ubuntu 22.04 and create your first ROS 2 package. This forms the foundation for all subsequent robotics development in this book.

## Learning Objectives

After completing this lab, you will be able to:
- Install ROS 2 Humble Hawksbill on Ubuntu 22.04
- Create and configure a ROS 2 workspace
- Create a custom ROS 2 package
- Build and source the workspace
- Verify the installation with basic commands

## Prerequisites

- Ubuntu 22.04 LTS installed
- Administrative access to install packages
- Internet connection for package downloads

## Estimated Duration

1-2 hours depending on system specifications and internet speed.

## Step-by-Step Instructions

### Step 1: Install System Dependencies

First, update your system and install basic development tools:

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install basic development tools
sudo apt install -y build-essential cmake git python3-pip python3-dev
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo apt install -y python3-vcstool wget curl gnupg lsb-release
```

### Step 2: Set Up Locale

Ensure locale is properly configured for ROS 2:

```bash
# Set up locale
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8
```

### Step 3: Install ROS 2 Humble Hawksbill

Add the ROS 2 repository and install the desktop package:

```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-dev-tools

# Install Python packages
pip3 install -U argcomplete
```

### Step 4: Source ROS 2 Environment

Add ROS 2 to your environment for the current session and permanently:

```bash
# Source for current session
source /opt/ros/humble/setup.bash

# Add to bashrc for permanent setup
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Step 5: Create ROS 2 Workspace

Create a workspace directory structure:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

### Step 6: Create a Custom Package

Create a simple package for the humanoid robot examples:

```bash
# Navigate to source directory
cd ~/ros2_ws/src

# Create a package for basic examples
ros2 pkg create --build-type ament_python ros2_nervous_system_examples --dependencies rclpy std_msgs

# Alternative for C++ package
# ros2 pkg create --build-type ament_cmake cpp_examples --dependencies rclcpp std_msgs
```

### Step 7: Build the Workspace

Build the workspace using colcon:

```bash
# Go back to workspace root
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the built workspace
source install/setup.bash
```

### Step 8: Verify Installation

Test that everything is working correctly:

```bash
# Check ROS 2 version
ros2 --version

# List available packages
ros2 pkg list | grep ros2_nervous_system_examples

# Check available topics (should be empty initially)
ros2 topic list
```

### Step 9: Create a Simple Publisher Node

Create a basic publisher in your new package:

```bash
# Navigate to the package directory
cd ~/ros2_ws/src/ros2_nervous_system_examples/ros2_nervous_system_examples

# Create a simple publisher script
cat << 'EOF' > talker.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

### Step 10: Make the Script Executable and Test

```bash
# Make the script executable
chmod +x ~/ros2_ws/src/ros2_nervous_system_examples/ros2_nervous_system_examples/talker.py

# Build the workspace again
cd ~/ros2_ws
colcon build
source install/setup.bash

# Test the publisher (in a separate terminal, run: ros2 run ros2_nervous_system_examples talker)
```

## Expected Output

When you run the talker node, you should see output similar to:

```
[INFO] [1681234567.123456789] [talker]: Publishing: "Hello World: 0"
[INFO] [1681234567.623456789] [talker]: Publishing: "Hello World: 1"
[INFO] [1681234568.123456789] [talker]: Publishing: "Hello World: 2"
...
```

## Troubleshooting Tips

1. **Package not found errors**: Ensure you've sourced both `/opt/ros/humble/setup.bash` and `~/ros2_ws/install/setup.bash`

2. **Permission denied errors**: Check file permissions and ensure you have write access to the workspace directory

3. **Build errors**: Verify all dependencies are installed and check for typos in package names

4. **Network issues**: If downloads are failing, check your internet connection and proxy settings if applicable

5. **Python path issues**: Ensure you're using Python 3 and that the ROS 2 Python packages are accessible

## Validation

To validate that you've completed this lab successfully:

1. You can run `ros2 run ros2_nervous_system_examples talker` without errors
2. You can see the published messages in the terminal
3. Your workspace builds successfully with `colcon build`
4. The `ros2_nervous_system_examples` package appears in `ros2 pkg list`

## Next Steps

After completing this lab, you should:
- Proceed to [Lab 2: Communication Patterns](./lab2-communication.md)
- Review the concepts of nodes, topics, and messages in the main module content
- Practice creating additional simple ROS 2 nodes to reinforce the concepts

## Summary

This lab established the foundational ROS 2 development environment and created your first ROS 2 package with a simple publisher node. You now have a working ROS 2 workspace that will serve as the foundation for all subsequent robotics development in this book.