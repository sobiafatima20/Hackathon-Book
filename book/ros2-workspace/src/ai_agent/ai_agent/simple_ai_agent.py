import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np


class SimpleAIAgentNode(Node):
    def __init__(self):
        super().__init__('simple_ai_agent')

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
        self.target_acquired = False

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = np.array(msg.ranges)
        # Simple obstacle detection
        if self.scan_data is not None and len(self.scan_data) > 0:
            valid_distances = self.scan_data[np.isfinite(self.scan_data)]
            if len(valid_distances) > 0:
                self.obstacle_detected = np.min(valid_distances) < 0.5

    def decision_loop(self):
        """Main AI decision-making loop"""
        if self.scan_data is None:
            return

        cmd_msg = Twist()

        if self.obstacle_detected:
            # Obstacle avoidance behavior
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5  # Turn right to avoid
            status_msg = "Avoiding obstacle"
        else:
            # Forward movement with exploration
            cmd_msg.linear.x = 0.2
            cmd_msg.angular.z = 0.0
            status_msg = "Exploring environment"

        # Publish command
        self.cmd_vel_pub.publish(cmd_msg)

        # Publish AI status
        status = String()
        status.data = status_msg
        self.ai_status_pub.publish(status)

        self.get_logger().info(f'AI Decision: {status_msg}')


def main(args=None):
    rclpy.init(args=args)
    ai_agent = SimpleAIAgentNode()
    rclpy.spin(ai_agent)
    ai_agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()