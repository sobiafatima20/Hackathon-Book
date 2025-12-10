# Gazebo Setup and Configuration

## Overview

This section covers setting up Gazebo Garden for physics-based simulation of humanoid robots. Gazebo provides realistic sensor simulation and environment modeling capabilities that are essential for robotics development.

## Installing Gazebo Garden

### System Requirements

- Ubuntu 22.04 LTS
- Graphics card with OpenGL 3.3+ support
- At least 8GB RAM (16GB+ recommended)
- 5GB+ free disk space

### Installation Steps

```bash
# Install Gazebo Garden (recommended version for ROS 2 Humble)
sudo apt install -y gz-garden

# Alternative: Install Gazebo Classic if preferred
# sudo apt install -y gazebo libgazebo-dev
```

### Verify Installation

```bash
# Check Gazebo version
gz --version

# Launch Gazebo GUI
gz sim
```

## Basic Gazebo Concepts

### Worlds
Gazebo worlds define the environment where robots operate, including:
- Physics properties (gravity, air resistance)
- Lighting conditions
- Static objects and obstacles
- Terrain properties

### Models
Robot and object models in Gazebo:
- Defined in SDF (Simulation Description Format)
- Include visual, collision, and inertial properties
- Can contain plugins for custom behavior

### Plugins
Gazebo plugins extend simulation capabilities:
- Sensor plugins for realistic data generation
- Controller plugins for robot actuation
- GUI plugins for custom interfaces

## SDF vs URDF

While URDF is used in ROS for robot description, Gazebo uses SDF (Simulation Description Format). However, you can convert URDF to SDF for use in Gazebo:

```xml
<!-- Example SDF World -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_humanoid_world">
    <!-- Physics properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
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

    <!-- Sample robot model -->
    <include>
      <uri>model://simple_humanoid</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>
  </world>
</sdf>
```

## Gazebo-ROS 2 Integration

### Installation

```bash
# Install Gazebo ROS 2 packages
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-dev
```

### Launching Gazebo with ROS 2

```bash
# Launch Gazebo with ROS 2 bridge
ros2 launch gazebo_ros gazebo.launch.py

# Or launch with a specific world
ros2 launch gazebo_ros gazebo.launch.py world:=path/to/world.sdf
```

## Sensor Simulation

Gazebo can simulate various sensors:

### Laser Scanner (LiDAR)
```xml
<sensor name="lidar" type="ray">
  <pose>0.1 0 0.1 0 0 0</pose>
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
  <plugin filename="libgazebo_ros_ray_sensor.so" name="gazebo_ros_head_lidar">
    <ros>
      <namespace>laser</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### IMU Sensor
```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <pose>0 0 0 0 0 0</pose>
  <plugin filename="libgazebo_ros_imu.so" name="imu_plugin">
    <ros>
      <namespace>imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</sensor>
```

### Camera Sensor
```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
    <ros>
      <namespace>camera</namespace>
    </ros>
    <camera_name>rgb_camera</camera_name>
    <image_topic_name>image_raw</image_topic_name>
    <camera_info_topic_name>camera_info</camera_info_topic_name>
  </plugin>
</sensor>
```

## Practical Example: Humanoid Robot in Gazebo

```
ALGORITHM: Gazebo_Sensor_Simulation
INPUT: robot_model, sensor_config
OUTPUT: sensor_data_stream

BEGIN
    physics_engine = initialize_physics_engine("ODE")
    robot = load_robot_model(robot_model)

    FOR each sensor IN sensor_config:
        IF sensor.type == "LiDAR":
            sensor_plugin = create_lidar_plugin(sensor.parameters)
        ELSE IF sensor.type == "IMU":
            sensor_plugin = create_imu_plugin(sensor.parameters)
        ELSE IF sensor.type == "Camera":
            sensor_plugin = create_camera_plugin(sensor.parameters)
        END IF
        robot.attach_sensor(sensor_plugin)
    END FOR

    WHILE simulation_running:
        physics_engine.step()
        FOR each sensor IN robot.sensors:
            sensor_data = sensor_plugin.get_data()
            publish_sensor_data(sensor_data)
        END FOR
    END WHILE
END
```

## Performance Optimization

### Physics Settings
- Adjust `max_step_size` for simulation accuracy vs performance
- Tune `real_time_factor` for real-time vs faster-than-real-time simulation
- Use appropriate solver parameters for your use case

### Graphics Settings
- Configure rendering quality based on available GPU resources
- Use Level of Detail (LOD) for complex models
- Optimize texture sizes for better performance

## Troubleshooting

### Common Issues
1. **Graphics errors**: Ensure proper graphics drivers are installed
2. **Model not loading**: Verify SDF/URDF syntax and file paths
3. **Sensor data not publishing**: Check ROS 2 bridge connection
4. **Performance issues**: Reduce physics step size or simplify models

## Next Steps

Continue to [Unity Integration for Visualization](./unity-integration.md) to learn about visualizing your robot in Unity.