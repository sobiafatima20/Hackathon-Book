# AI Integration with ROS 2

## Overview

This section covers how to integrate artificial intelligence agents with the ROS 2 communication framework. The integration allows AI systems to perceive sensor data, make decisions, and execute actions through the robotic nervous system.

## AI Agent Architecture

An AI agent in the ROS 2 context typically consists of:
- **Perception module**: Processes sensor data from ROS topics
- **Decision module**: Implements AI algorithms for planning and reasoning
- **Action module**: Publishes commands to control the robot
- **Communication module**: Interfaces with ROS 2 middleware

## Python Implementation with rclpy

### Basic AI Agent Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent')

        # Subscribers for sensor data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Publisher for robot commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Publisher for AI decisions
        self.ai_status_pub = self.create_publisher(
            String,
            'ai_status',
            10
        )

        # Timer for decision-making loop
        self.timer = self.create_timer(0.1, self.decision_loop)

        # Internal state
        self.scan_data = None
        self.obstacle_detected = False

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = np.array(msg.ranges)
        # Simple obstacle detection
        if self.scan_data is not None:
            self.obstacle_detected = np.min(self.scan_data) < 0.5

    def decision_loop(self):
        """Main AI decision-making loop"""
        if self.scan_data is None:
            return

        cmd_msg = Twist()

        if self.obstacle_detected:
            # Obstacle avoidance behavior
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5  # Turn
            status_msg = "Avoiding obstacle"
        else:
            # Forward movement
            cmd_msg.linear.x = 0.2
            cmd_msg.angular.z = 0.0
            status_msg = "Moving forward"

        self.cmd_vel_pub.publish(cmd_msg)

        status = String()
        status.data = status_msg
        self.ai_status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()
    rclpy.spin(ai_agent)
    ai_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## AI Integration Patterns

### 1. Reactive AI Pattern
Simple, immediate responses to sensor inputs without complex planning.

### 2. Deliberative AI Pattern
Planning-based approach with explicit goal representation and pathfinding.

### 3. Learning-based AI Pattern
Adaptive systems that improve performance through experience.

## Machine Learning Integration

### TensorFlow/Keras with ROS 2

```python
import tensorflow as tf
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MLPerceptionNode(Node):
    def __init__(self):
        super().__init__('ml_perception')
        self.model = tf.keras.models.load_model('path/to/model.h5')
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.result_pub = self.create_publisher(
            String,
            'ml_result',
            10
        )

    def image_callback(self, msg):
        """Process image with ML model"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Preprocess image for model
        input_tensor = tf.convert_to_tensor(cv_image)
        input_tensor = tf.expand_dims(input_tensor, 0)

        # Run inference
        predictions = self.model(input_tensor)
        result = tf.argmax(predictions, axis=1)

        # Publish result
        result_msg = String()
        result_msg.data = f'Class: {result.numpy()[0]}'
        self.result_pub.publish(result_msg)
```

### PyTorch Integration

```python
import torch
import torchvision.transforms as transforms
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class PyTorchPerceptionNode(Node):
    def __init__(self):
        super().__init__('pytorch_perception')
        self.model = torch.load('path/to/model.pth')
        self.model.eval()

        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((224, 224))
        ])

        # ROS subscriptions and publishers
        # ... (similar to TensorFlow example)
```

## AI Decision Making Algorithms

### State Machine Implementation

```
ALGORITHM: AI_State_Machine
INPUT: sensor_data, current_state
OUTPUT: robot_action

BEGIN
    STATE_IDLE:
        IF sensor_data.detects_object():
            transition_to(STATE_APPROACH)
        ELSE IF sensor_data.detects_threat():
            transition_to(STATE_AVOID)
        END IF

    STATE_APPROACH:
        execute_approach_behavior()
        IF distance_to_object < threshold:
            transition_to(STATE_MANIPULATE)
        END IF

    STATE_AVOID:
        execute_avoidance_behavior()
        IF threat_cleared():
            transition_to(STATE_IDLE)
        END IF

    STATE_MANIPULATE:
        execute_manipulation_behavior()
        transition_to(STATE_IDLE)
END
```

### Behavior Tree Integration

Behavior trees provide a hierarchical approach to AI decision making:

```python
class BehaviorNode:
    def tick(self):
        pass

class SequenceNode(BehaviorNode):
    def __init__(self, children):
        self.children = children

    def tick(self):
        for child in self.children:
            result = child.tick()
            if result != 'SUCCESS':
                return result
        return 'SUCCESS'

class SelectorNode(BehaviorNode):
    def __init__(self, children):
        self.children = children

    def tick(self):
        for child in self.children:
            result = child.tick()
            if result != 'FAILURE':
                return result
        return 'FAILURE'
```

## ROS 2 AI Integration Best Practices

### 1. Message Rate Management
- Use appropriate QoS settings for AI processing
- Implement message filtering for high-frequency sensors
- Consider message throttling for computationally expensive AI tasks

### 2. Computational Efficiency
- Offload heavy computation to separate nodes
- Use multi-threading when appropriate
- Implement caching for repeated computations

### 3. Error Handling
- Implement fallback behaviors when AI fails
- Monitor AI node health
- Provide graceful degradation

### 4. Performance Monitoring
- Track AI processing time
- Monitor memory usage
- Log AI decision accuracy

## AI-ROS Integration Example

```
ALGORITHM: ROS2_AI_Agent
INPUT: sensor_stream
OUTPUT: action_commands

BEGIN
    ai_node = create_node("ai_agent")

    // Subscribe to sensor streams
    sensor_sub = create_subscription(
        SensorStream,
        "sensor_fusion",
        process_sensors,
        QOS_PROFILE_SENSORS
    )

    // Publish action commands
    action_pub = create_publisher(
        ActionCommand,
        "ai_actions",
        QOS_PROFILE_COMMANDS
    )

    FUNCTION process_sensors(sensor_data):
        perception_result = run_perception_pipeline(sensor_data)
        decision = make_decision(perception_result)
        command = generate_action_command(decision)
        action_pub.publish(command)
    END FUNCTION

    spin(ai_node)
END
```

## Safety Considerations

When integrating AI with physical robots, consider:
- Safety limits and constraints
- Emergency stop mechanisms
- Validation of AI outputs
- Redundant safety systems
- Human oversight capabilities

## Next Steps

Continue to [Module 1 References](./references.md) to review the citations and resources for this module.