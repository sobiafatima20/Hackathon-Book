import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np


class HumanoidControllerNode(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

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

        # Publisher for controller status
        self.status_pub = self.create_publisher(
            String,
            'controller_status',
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.scan_data = None
        self.obstacle_detected = False
        self.safe_to_move = True

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = np.array(msg.ranges)
        # Check for obstacles in front of the robot
        if self.scan_data is not None:
            # Consider the front 60 degrees (30 degrees on each side)
            front_scan = self.scan_data[len(self.scan_data)//2 - 30:len(self.scan_data)//2 + 30]
            # Remove invalid readings (inf or nan)
            valid_distances = front_scan[np.isfinite(front_scan)]
            if len(valid_distances) > 0:
                min_distance = np.min(valid_distances)
                self.obstacle_detected = min_distance < 0.5
                self.safe_to_move = min_distance > 0.8

    def control_loop(self):
        """Main control loop"""
        if self.scan_data is None:
            return

        cmd_msg = Twist()

        if self.obstacle_detected and not self.safe_to_move:
            # Stop and turn to avoid obstacle
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5  # Turn right
            status_msg = "Obstacle detected, turning to avoid"
        elif self.safe_to_move:
            # Move forward safely
            cmd_msg.linear.x = 0.2
            cmd_msg.angular.z = 0.0
            status_msg = "Moving forward safely"
        else:
            # Stop and assess
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
            status_msg = "Assessing path"

        # Publish command
        self.cmd_vel_pub.publish(cmd_msg)

        # Publish status
        status = String()
        status.data = status_msg
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()