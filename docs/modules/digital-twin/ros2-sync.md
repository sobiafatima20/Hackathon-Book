# ROS 2 Synchronization

## Overview

This section covers the synchronization between Gazebo simulation and ROS 2, which is critical for realistic robot simulation and testing. Proper synchronization ensures that sensor data, robot states, and control commands are properly coordinated between the simulation and the ROS 2 system.

## Time Synchronization

### Simulation Time vs Real Time

In Gazebo, time can run at different rates compared to real time:
- **Real-time factor**: Controls how fast simulation time progresses relative to real time
- **Fixed step size**: Determines the physics simulation time step
- **Clock publishing**: Gazebo can publish simulation time to ROS 2

### Time Source Configuration

```xml
<!-- In your world file -->
<world name="simulation_world">
  <!-- Physics properties -->
  <physics type="ode">
    <max_step_size>0.001</max_step_size>  <!-- 1ms physics step -->
    <real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation -->
    <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz update rate -->
  </physics>

  <!-- Enable simulation clock publishing -->
  <include>
    <uri>model://ground_plane</uri>
  </include>
</world>
```

### Using Simulation Time in ROS 2 Nodes

```python
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.time import Time as RclpyTime
from rclpy.clock import ClockType

class TimeSyncNode(Node):
    def __init__(self):
        super().__init__('time_sync_node')

        # Subscribe to simulation time
        self.time_sub = self.create_subscription(
            Time,
            '/clock',  # Gazebo publishes simulation time here
            self.clock_callback,
            10
        )

        # Use simulation time in your node
        self.use_sim_time_param = self.declare_parameter('use_sim_time', True)

    def clock_callback(self, msg):
        sim_time = RclpyTime.from_msg(msg)
        self.get_logger().info(f'Simulation time: {sim_time.nanoseconds / 1e9:.3f}s')
```

## Robot State Synchronization

### Joint State Publisher

The robot state publisher synchronizes joint positions between Gazebo and ROS 2:

```xml
<!-- In your robot's URDF/SDF -->
<ros>
  <namespace>/humanoid_robot</namespace>
  <remapping>joint_states:=/humanoid_robot/joint_states</remapping>
</ros>
```

### State Publisher Configuration

```python
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        self.joint_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz

    def publish_joint_states(self):
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3']  # Your joint names
        msg.position = [0.0, 0.0, 0.0]  # Get actual positions from Gazebo
        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_publisher.publish(msg)
```

## Sensor Data Synchronization

### Time-Stamped Messages

All sensor data should be properly time-stamped for synchronization:

```python
from sensor_msgs.msg import LaserScan
from builtin_interfaces.msg import Time

def publish_laser_scan(self, ranges):
    scan_msg = LaserScan()
    scan_msg.header.stamp = self.get_clock().now().to_msg()  # Current time
    scan_msg.header.frame_id = 'laser_link'

    scan_msg.angle_min = -math.pi
    scan_msg.angle_max = math.pi
    scan_msg.angle_increment = 2 * math.pi / len(ranges)
    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 0.1  # Time between scans
    scan_msg.range_min = 0.1
    scan_msg.range_max = 30.0

    scan_msg.ranges = ranges
    scan_msg.intensities = []  # Optional

    self.laser_publisher.publish(scan_msg)
```

### Message Filters for Multi-Sensor Synchronization

Use ROS 2 message filters to synchronize data from multiple sensors:

```python
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sensor_msgs.msg as sensor_msgs

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Create subscribers
        self.laser_sub = Subscriber(self, LaserScan, '/laser_scan')
        self.imu_sub = Subscriber(self, Imu, '/imu/data')
        self.camera_sub = Subscriber(self, Image, '/camera/image_raw')

        # Synchronize messages with time tolerance
        self.ats = ApproximateTimeSynchronizer(
            [self.laser_sub, self.imu_sub, self.camera_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.ats.registerCallback(self.sync_callback)

    def sync_callback(self, laser_msg, imu_msg, camera_msg):
        # Process synchronized sensor data
        self.get_logger().info(f'Synchronized at time: {laser_msg.header.stamp}')
        # Perform sensor fusion
```

## Control Command Synchronization

### Command Timing

When sending control commands to simulated robots, timing is crucial:

```python
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Velocity command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Joint position command publisher
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Control loop timer
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz

    def control_loop(self):
        # Get current time
        current_time = self.get_clock().now()

        # Calculate control commands based on current state
        cmd_vel = self.calculate_velocity_command()

        # Publish commands with timestamp
        cmd_vel.header.stamp = current_time.to_msg()
        self.cmd_vel_pub.publish(cmd_vel)
```

## Quality of Service for Synchronization

### Appropriate QoS Settings

Different synchronization needs require different QoS settings:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# For critical control commands
control_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# For sensor data (frequent updates)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10  # Keep last 10 messages
)

# For state information that should persist
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

## Gazebo-ROS 2 Bridge Configuration

### Launch File for Complete Integration

```python
# launch/gazebo_ros_integration.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Launch Gazebo with world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'my_world.sdf'],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description':
                Command(['xacro', ' ', FindFile('robot_description', 'urdf/robot.xacro')])
            }
        ]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        joint_state_publisher
    ])
```

## Troubleshooting Synchronization Issues

### Common Problems and Solutions

1. **Time drift between simulation and ROS**: Ensure `use_sim_time` parameter is set correctly on all nodes

2. **Delayed sensor data**: Check network configuration and message queue sizes

3. **Control command lag**: Reduce physics step size or use faster control loop rates

4. **Message loss**: Increase QoS depth or switch to RELIABLE reliability

### Debugging Tools

```bash
# Monitor clock messages
ros2 topic echo /clock

# Check message delays
ros2 topic hz /sensor_topic_name

# Monitor all topics
ros2 topic list

# Check node connections
ros2 run rqt_graph rqt_graph
```

## Performance Considerations

### Optimizing Synchronization

1. **Physics step size**: Balance accuracy with performance (typically 1ms for real-time)

2. **Update rates**: Match simulation update rates to sensor/actuator capabilities

3. **Message frequency**: Use appropriate rates for different sensor types

4. **Data processing**: Optimize sensor data processing to avoid bottlenecks

## Advanced Synchronization Patterns

### Event-Based Synchronization

```python
class EventBasedSync(Node):
    def __init__(self):
        super().__init__('event_sync')

        self.sensor_queue = []
        self.event_threshold = 0.5  # Process when sensor value exceeds threshold

    def sensor_callback(self, msg):
        self.sensor_queue.append(msg)

        if self.should_process_event(msg):
            self.process_event()

    def should_process_event(self, msg):
        # Custom logic for event detection
        return max(msg.ranges) < self.event_threshold
```

## Next Steps

Continue to [Module 2 References](./references.md) to review the citations and resources for this module, or proceed to [Module 3: AI-Robot Brain](../../modules/ai-robot-brain/index.md).