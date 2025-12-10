#!/bin/bash

# Test script to verify the ROS 2 workspace can be built
echo "Testing ROS 2 workspace build..."

# Check if we're in the right directory
if [ ! -f "src/humanoid_description/package.xml" ]; then
    echo "Error: Not in the correct workspace directory"
    exit 1
fi

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "ROS 2 Humble sourced successfully"
else
    echo "Error: ROS 2 Humble not found. Please install ROS 2 Humble Hawksbill."
    exit 1
fi

# Try to build the workspace
echo "Building workspace..."
colcon build --packages-select humanoid_description ros2_nervous_system_examples ai_agent

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Sourcing the workspace..."
    source install/setup.bash
    echo "Workspace sourced successfully"
    echo "You can now run examples with commands like:"
    echo "  ros2 run ros2_nervous_system_examples talker"
    echo "  ros2 launch humanoid_description humanoid_description.launch.py"
else
    echo "Build failed. Please check the error messages above."
    exit 1
fi