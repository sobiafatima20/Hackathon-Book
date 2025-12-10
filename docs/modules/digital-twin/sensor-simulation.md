# Sensor Simulation in Gazebo

## Overview

This section covers configuring and using various sensors in Gazebo simulation for humanoid robots. Proper sensor simulation is crucial for developing and testing perception algorithms before deployment on real hardware.

## Sensor Types in Gazebo

Gazebo supports simulation of various sensor types that are commonly found on robots:

### Range Sensors
- **LiDAR**: Simulates laser range finders
- **Sonar**: Simulates ultrasonic distance sensors
- **IR Sensors**: Simulates infrared distance sensors

### Vision Sensors
- **RGB Cameras**: Simulates color cameras
- **Depth Cameras**: Simulates depth-sensing cameras
- **Stereo Cameras**: Simulates stereo vision systems

### Inertial Sensors
- **IMU**: Simulates Inertial Measurement Units
- **Accelerometers**: Simulates acceleration sensors
- **Gyroscopes**: Simulates angular velocity sensors

### Force/Torque Sensors
- **Force/Torque Sensors**: Simulates force and torque measurements
- **Contact Sensors**: Simulates contact detection

## LiDAR Configuration

### Basic LiDAR Setup

```xml
<sensor name="lidar" type="ray">
  <pose>0.1 0 0.1 0 0 0</pose> <!-- Position on robot -->
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples> <!-- Number of beams -->
        <resolution>1</resolution> <!-- Resolution of beams -->
        <min_angle>-3.14159</min_angle> <!-- -π radians -->
        <max_angle>3.14159</max_angle> <!-- π radians -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min> <!-- Minimum detectable range -->
      <max>30.0</max> <!-- Maximum detectable range -->
      <resolution>0.01</resolution> <!-- Range resolution -->
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

### Multi-layer LiDAR (3D LiDAR)

```xml
<sensor name="velodyne" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>800</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.2618</min_angle> <!-- -15 degrees -->
        <max_angle>0.2618</max_angle>   <!-- 15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.2</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin filename="libgazebo_ros_velodyne_gpu.so" name="gazebo_ros_velodyne_controller">
    <ros>
      <namespace>velodyne</namespace>
    </ros>
    <min_range>0.2</min_range>
    <max_range>100.0</max_range>
    <gaussian_noise>0.008</gaussian_noise>
  </plugin>
</sensor>
```

## Camera Configuration

### RGB Camera

```xml
<sensor name="rgb_camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
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

### Depth Camera

```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>10</far>
    </clip>
  </camera>
  <plugin filename="libgazebo_ros_openni_kinect.so" name="camera_controller">
    <ros>
      <namespace>depth_camera</namespace>
    </ros>
    <camera_name>depth_camera</camera_name>
    <image_topic_name>image_raw</image_topic_name>
    <depth_image_topic_name>depth/image_raw</depth_image_topic_name>
    <point_cloud_topic_name>points</point_cloud_topic_name>
    <camera_info_topic_name>camera_info</camera_info_topic_name>
    <depth_image_camera_info_topic_name>depth/camera_info</depth_image_camera_info_topic_name>
    <frame_name>depth_camera_frame</frame_name>
    <baseline>0.1</baseline>
    <distortion_k1>0.0</distortion_k1>
    <distortion_k2>0.0</distortion_k2>
    <distortion_k3>0.0</distortion_k3>
    <distortion_t1>0.0</distortion_t1>
    <distortion_t2>0.0</distortion_t2>
  </plugin>
</sensor>
```

## IMU Configuration

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <pose>0 0 0 0 0 0</pose> <!-- Mount position -->
  <plugin filename="libgazebo_ros_imu.so" name="imu_plugin">
    <ros>
      <namespace>imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
    <topic>imu/data</topic>
    <body_name>imu_link</body_name>
    <update_rate>100</update_rate>
    <gaussian_noise>0.001</gaussian_noise>
  </plugin>
</sensor>
```

## Sensor Noise and Realism

### Adding Noise Models

Real sensors have inherent noise and inaccuracies. Gazebo allows you to model this:

```xml
<sensor name="noisy_lidar" type="ray">
  <ray>
    <!-- ... ray configuration ... -->
  </ray>
  <plugin filename="libgazebo_ros_ray_sensor.so" name="noisy_lidar_plugin">
    <gaussian_noise>0.01</gaussian_noise> <!-- 1cm noise -->
    <ros>
      <namespace>laser</namespace>
    </ros>
  </plugin>
</sensor>
```

### Noise Parameters by Sensor Type

| Sensor Type | Typical Noise | Configuration |
|-------------|---------------|---------------|
| LiDAR | 0.5-2cm | `gaussian_noise` |
| Camera | Pixel noise | Camera plugin parameters |
| IMU | 0.001-0.01 rad/s | `gaussian_noise` |
| GPS | 1-3m | Specialized GPS plugins |

## Sensor Fusion in Simulation

### Multi-Sensor Configuration Example

```xml
<!-- Humanoid head with multiple sensors -->
<joint name="head_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="head_camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>

<link name="head_camera_link">
  <visual>
    <geometry>
      <cylinder radius="0.02" length="0.04"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.02" length="0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>

  <!-- RGB Camera -->
  <sensor name="head_camera" type="camera">
    <!-- Camera configuration -->
  </sensor>

  <!-- IMU -->
  <sensor name="head_imu" type="imu">
    <!-- IMU configuration -->
  </sensor>
</link>
```

## Sensor Data Processing Pipeline

```
ALGORITHM: Sensor_Data_Pipeline
INPUT: raw_sensor_data
OUTPUT: processed_sensor_data

BEGIN
    // Collect raw data from all sensors
    lidar_data = get_lidar_scan()
    camera_data = get_camera_image()
    imu_data = get_imu_readings()
    depth_data = get_depth_image()

    // Process each sensor stream
    processed_lidar = filter_lidar_data(lidar_data)
    processed_camera = rectify_camera_image(camera_data)
    processed_imu = calibrate_imu_data(imu_data)
    processed_depth = process_depth_data(depth_data)

    // Perform sensor fusion
    fused_data = fuse_sensor_data(
        processed_lidar,
        processed_camera,
        processed_imu,
        processed_depth
    )

    // Publish processed data
    publish_sensor_data(fused_data)

END
```

## Performance Optimization

### Sensor Update Rates

Different sensors require different update rates:

- **LiDAR**: 5-20 Hz (depending on complexity)
- **Cameras**: 10-30 Hz
- **IMU**: 50-200 Hz
- **GPS**: 1-10 Hz

### Computational Considerations

- Use lower resolution for high-frequency sensors if performance is an issue
- Implement sensor message throttling for visualization-only sensors
- Consider using GPU-accelerated sensors where available

## Quality of Service for Sensor Data

When publishing sensor data in ROS 2, consider appropriate QoS settings:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# For critical sensors (e.g., collision avoidance)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# For visualization sensors
visualization_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

## Troubleshooting Sensor Simulation

### Common Issues

1. **No sensor data**: Check sensor plugin loading and ROS topic connections
2. **Poor performance**: Reduce sensor resolution or update rate
3. **Inaccurate data**: Verify sensor placement and noise parameters
4. **Synchronization issues**: Use appropriate time synchronization methods

### Debugging Commands

```bash
# Check if sensor topics are being published
ros2 topic list | grep sensor

# Monitor sensor data
ros2 topic echo /sensor_topic_name

# Check sensor plugin status
gz topic -i
```

## Next Steps

Continue to [ROS 2 Synchronization](./ros2-sync.md) to learn about synchronizing sensor data between Gazebo and ROS 2.