# Lab 1: Gazebo Environment Setup

## Overview

In this lab, you will install and configure Gazebo Garden for humanoid robot simulation. You will create a basic simulation environment and test it with a simple robot model.

## Learning Objectives

After completing this lab, you will be able to:
- Install Gazebo Garden on Ubuntu 22.04
- Create a basic simulation world
- Load a robot model into Gazebo
- Configure basic sensors in the simulation
- Launch Gazebo with ROS 2 integration

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed (from Module 1)
- Graphics card with OpenGL 3.3+ support
- At least 8GB RAM

## Estimated Duration

2-3 hours depending on system specifications and download speeds.

## Step-by-Step Instructions

### Step 1: Install Gazebo Garden

```bash
# Update package list
sudo apt update

# Install Gazebo Garden
sudo apt install -y gz-garden

# Verify installation
gz --version
```

### Step 2: Install Gazebo ROS 2 Packages

```bash
# Install Gazebo ROS 2 packages
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev

# Verify installation
dpkg -l | grep gazebo-ros
```

### Step 3: Create a Basic World File

Create a simple world file for your humanoid robot:

```bash
# Create directory for worlds
mkdir -p ~/ros2_ws/src/humanoid_description/worlds
cd ~/ros2_ws/src/humanoid_description/worlds

# Create a basic world file
cat << 'EOF' > simple_humanoid_world.sdf
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_humanoid_world">
    <!-- Physics properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.3 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sample box obstacle -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="box_link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
EOF
```

### Step 4: Create a Simple Humanoid Model

Create a basic humanoid model in SDF format:

```bash
# Create models directory
mkdir -p ~/ros2_ws/src/humanoid_description/models/simple_humanoid

# Create the model configuration
cat << 'EOF' > ~/ros2_ws/src/humanoid_description/models/simple_humanoid/model.config
<?xml version="1.0"?>
<model>
  <name>simple_humanoid</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Physical AI Book</name>
    <email>info@physicalai-book.org</email>
  </author>
  <description>A simple humanoid robot model for simulation.</description>
</model>
EOF
```

```bash
# Create the model file
cat << 'EOF' > ~/ros2_ws/src/humanoid_description/models/simple_humanoid/model.sdf
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_humanoid">
    <link name="base_link">
      <pose>0 0 1 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.7 0.3 0.3 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.2</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.3</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
    </link>

    <link name="head">
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
      </collision>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
    </link>

    <joint name="neck_joint" type="revolute">
      <parent>base_link</parent>
      <child>head</child>
      <pose>0 0 0.4 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.5</upper>
          <effort>10</effort>
          <velocity>2</velocity>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
EOF
```

### Step 5: Launch Gazebo with Your World

```bash
# Set up environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Launch Gazebo with your world
gz sim -r -v 4 ~/ros2_ws/src/humanoid_description/worlds/simple_humanoid_world.sdf
```

### Step 6: Add a LiDAR Sensor to the Robot

Update the humanoid model to include a LiDAR sensor:

```bash
# Create updated model with LiDAR
cat << 'EOF' > ~/ros2_ws/src/humanoid_description/models/simple_humanoid/model.sdf
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="simple_humanoid">
    <link name="base_link">
      <pose>0 0 1 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.7 0.3 0.3 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.5</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.2</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.3</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
    </link>

    <link name="head">
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.15</radius>
          </sphere>
        </geometry>
      </collision>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
    </link>

    <link name="lidar_link">
      <pose>0.1 0 0.1 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.2 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.02</radius>
            <length>0.04</length>
          </cylinder>
        </geometry>
      </collision>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>1e-5</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1e-5</iyy>
          <iyz>0</iyz>
          <izz>2e-5</izz>
        </inertia>
      </inertial>

      <sensor name="lidar" type="ray">
        <pose>0 0 0 0 0 0</pose>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
        </ray>
        <always_on>true</always_on>
        <update_rate>10</update_rate>
      </sensor>
    </link>

    <joint name="neck_joint" type="revolute">
      <parent>base_link</parent>
      <child>head</child>
      <pose>0 0 0.4 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.5</lower>
          <upper>0.5</upper>
          <effort>10</effort>
          <velocity>2</velocity>
        </limit>
      </axis>
    </joint>

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
      <pose>0.1 0 0.1 0 0 0</pose>
    </joint>
  </model>
</sdf>
EOF
```

### Step 7: Test ROS 2 Integration

Create a simple launch file to test Gazebo-ROS 2 integration:

```bash
# Create launch directory
mkdir -p ~/ros2_ws/src/humanoid_description/launch

# Create launch file
cat << 'EOF' > ~/ros2_ws/src/humanoid_description/launch/gazebo_humanoid.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='simple_humanoid_world.sdf')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                FindPackageShare('humanoid_description'),
                'worlds',
                world_name
            ]), '-r -v 4']
        }
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description':
                # We'll load this from a parameter server later
            }
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='simple_humanoid_world.sdf',
            description='Choose one of the world files from `/humanoid_description/worlds`'
        ),
        gazebo,
        robot_state_publisher
    ])
EOF
```

### Step 8: Build and Test

```bash
# Build the workspace
cd ~/ros2_ws
colcon build --packages-select humanoid_description
source install/setup.bash

# Launch Gazebo with the robot
ros2 launch humanoid_description gazebo_humanoid.launch.py
```

## Expected Output

When you run the simulation, you should see:
- Gazebo GUI with your simple humanoid robot in the world
- The robot positioned on the ground plane
- A box obstacle in the environment
- The robot should have a LiDAR sensor on its body
- The simulation should run at real-time speed

## Troubleshooting Tips

1. **Gazebo doesn't start**: Check graphics drivers and OpenGL support
2. **Model not loading**: Verify SDF syntax and file paths
3. **No sensor data**: Check sensor configuration and Gazebo ROS bridge
4. **Performance issues**: Reduce physics step size or simplify models
5. **ROS 2 connection issues**: Ensure proper environment setup

## Validation

To validate that you've completed this lab successfully:

1. Gazebo launches with your custom world and robot model
2. The robot appears in the simulation environment
3. The LiDAR sensor is visible on the robot model
4. You can interact with the simulation (move the robot, obstacles)
5. The simulation runs at approximately real-time speed

## Next Steps

After completing this lab, you should:
- Proceed to [Lab 2: Sensor Configuration](./lab2-sensor-config.md)
- Review the concepts of sensor simulation in the main module content
- Experiment with different sensor types and configurations in Gazebo

## Summary

This lab established a basic Gazebo simulation environment with a humanoid robot model and LiDAR sensor. You now have a foundation for more complex simulation scenarios and sensor integration that will be essential for the rest of Module 2.