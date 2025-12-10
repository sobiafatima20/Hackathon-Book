# Nodes and Topics Communication

## Overview

Nodes and topics form the foundation of ROS 2's publish-subscribe communication model. This pattern enables asynchronous communication between different parts of a robotic system, allowing for modular and scalable architectures.

## Node Implementation

### Python Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'topic', 10)
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
```

### C++ Implementation

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
```

## Topic Communication Patterns

### Publisher-Subscriber Pattern

The publisher-subscriber pattern is the most common communication method in ROS 2. Publishers send messages to topics without knowing who will receive them, and subscribers receive messages from topics without knowing who sent them.

### Quality of Service (QoS) Considerations

When implementing topic communication, consider these QoS settings:

- **Reliability Policy**: Choose between `RELIABLE` (all messages delivered) or `BEST_EFFORT` (best attempt delivery)
- **Durability Policy**: Choose between `TRANSIENT_LOCAL` (replay messages to late-joining subscribers) or `VOLATILE` (no replay)
- **History Policy**: Choose between `KEEP_ALL` or `KEEP_LAST` with a specified depth

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Example: Reliable communication with limited history
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

publisher = node.create_publisher(String, 'topic', qos_profile)
```

## Message Types and Custom Messages

ROS 2 provides standard message types in packages like `std_msgs`, `geometry_msgs`, and `sensor_msgs`. For specific applications, you can define custom message types using the `.msg` file format.

Example custom message (SensorData.msg):
```
float64 temperature
float64 humidity
bool motion_detected
```

## Advanced Communication Patterns

### Latched Topics

Latched topics retain the last published message and send it immediately to any new subscribers.

### Publisher with Callbacks

Implement callbacks to monitor publisher status and handle communication events.

## Practical Example: Sensor Data Pipeline

The following pseudocode demonstrates a complete sensor data pipeline:

```
ALGORITHM: Sensor_Data_Pipeline
INPUT: raw_sensor_readings
OUTPUT: processed_sensor_data

BEGIN
    sensor_node = create_node("sensor_processor")

    raw_sub = create_subscription(
        RawSensorMsg,
        "raw_sensor_data",
        process_raw_data,
        qos_profile=QOS_PROFILE_SENSOR_DATA
    )

    processed_pub = create_publisher(
        ProcessedSensorMsg,
        "processed_sensor_data",
        qos_profile=QOS_PROFILE_PROCESSED_DATA
    )

    FUNCTION process_raw_data(raw_msg):
        filtered_data = apply_filter(raw_msg.data)
        calibrated_data = calibrate_sensors(filtered_data)
        processed_msg = create_processed_message(calibrated_data)
        processed_pub.publish(processed_msg)
    END FUNCTION

    spin(sensor_node)
END
```

## Best Practices

1. Use appropriate QoS settings for your application requirements
2. Implement proper error handling and logging
3. Follow naming conventions (lowercase with underscores)
4. Use appropriate message types for data being exchanged
5. Consider message frequency and bandwidth requirements

## Next Steps

Continue to [URDF Robot Modeling](./urdf-modeling.md) to learn about describing robot models in ROS 2.