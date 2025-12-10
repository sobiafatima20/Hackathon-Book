# ROS 2 Architecture and Concepts

## Overview

ROS 2 (Robot Operating System 2) provides the communication infrastructure for robotic applications. It serves as the "nervous system" of a robot, enabling different components to exchange information and coordinate their actions.

## Key Concepts

### Nodes
A node is a process that performs computation. In ROS 2, nodes are designed to be modular and focused on a single task. Each node can publish or subscribe to messages, provide or use services, and manage parameters.

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello from ROS 2!')
```

### Topics and Messages
Topics are named buses over which nodes exchange messages. Messages are data structures that contain information being passed between nodes. Topics enable asynchronous communication through a publish-subscribe pattern.

### Services
Services provide synchronous request-response communication between nodes. A service client sends a request to a service server, which processes the request and returns a response.

### Actions
Actions are used for long-running tasks that may take some time to complete. They provide feedback during execution and can be preempted if needed.

### Parameters
Parameters are configuration values that can be set at runtime and changed dynamically. They allow for flexible configuration of nodes without recompilation.

## ROS 2 Architecture Components

### DDS (Data Distribution Service)
ROS 2 uses DDS as its underlying middleware. DDS provides the communication infrastructure that enables discovery, data delivery, and quality of service features.

### RMW (ROS Middleware)
The ROS Middleware layer abstracts the underlying DDS implementation, allowing ROS 2 to work with different DDS vendors.

### Client Libraries
ROS 2 provides client libraries for multiple programming languages, including:
- rclcpp for C++
- rclpy for Python
- rclrs for Rust (community maintained)

## Quality of Service (QoS) Settings

QoS settings allow you to specify requirements for communication reliability, durability, and performance:

- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient local
- **History**: Keep all or keep last N messages
- **Depth**: Size of the message queue

## Practical Example: Node Communication

The following pseudocode demonstrates the basic ROS 2 communication pattern:

```
ALGORITHM: ROS2_Humanoid_Node
INPUT: sensor_data
OUTPUT: actuator_commands

BEGIN
    node = create_node("humanoid_controller")
    sensor_sub = create_subscription(
        SensorMsg,
        "sensor_data",
        process_sensor_data,
        10
    )
    actuator_pub = create_publisher(
        ActuatorMsg,
        "actuator_commands",
        10
    )

    FUNCTION process_sensor_data(msg):
        processed = filter_and_analyze(msg.data)
        command = generate_actuator_command(processed)
        actuator_pub.publish(command)
    END FUNCTION

    spin(node)
END
```

## ROS 2 vs ROS 1

Key differences between ROS 2 and ROS 1 include:
- DDS-based middleware instead of custom TCPROS/UDPROS
- Built-in security features
- Support for multiple DDS implementations
- Improved real-time capabilities
- Better support for cross-platform development

## Next Steps

Continue to [Nodes and Topics Communication](./nodes-topics.md) to learn about implementing communication patterns in ROS 2.