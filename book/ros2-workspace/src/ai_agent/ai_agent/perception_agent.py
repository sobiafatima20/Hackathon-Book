import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np


class PerceptionAIAgentNode(Node):
    def __init__(self):
        super().__init__('perception_ai_agent')

        # Initialize CV bridge for image processing
        self.bridge = CvBridge()

        # Subscribers for sensor data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
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
        self.image_data = None
        self.obstacle_detected = False
        self.target_detected = False
        self.target_x = None

    def scan_callback(self, msg):
        """Process laser scan data"""
        self.scan_data = np.array(msg.ranges)
        # Simple obstacle detection
        if self.scan_data is not None and len(self.scan_data) > 0:
            valid_distances = self.scan_data[np.isfinite(self.scan_data)]
            if len(valid_distances) > 0:
                self.obstacle_detected = np.min(valid_distances) < 0.5

    def image_callback(self, msg):
        """Process image data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_data = cv_image
            # Simple color-based target detection (red object)
            self.detect_red_object(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def detect_red_object(self, image):
        """Detect red objects in the image"""
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define range for red color
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:  # Minimum area threshold
                self.target_detected = True
                # Get the center of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    self.target_x = int(M["m10"] / M["m00"])
                else:
                    self.target_x = None
            else:
                self.target_detected = False
                self.target_x = None
        else:
            self.target_detected = False
            self.target_x = None

    def decision_loop(self):
        """Main AI decision-making loop with perception"""
        if self.scan_data is None:
            return

        cmd_msg = Twist()

        if self.obstacle_detected:
            # Obstacle avoidance behavior
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5  # Turn right to avoid
            status_msg = "Avoiding obstacle"
        elif self.target_detected and self.target_x is not None:
            # Navigate toward detected target
            image_center = 320  # Assuming 640x480 image
            error = self.target_x - image_center

            # Proportional control for target following
            cmd_msg.linear.x = 0.15  # Forward at reduced speed
            cmd_msg.angular.z = -error * 0.005  # Turn toward target
            status_msg = f"Following target at x={self.target_x}"
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
    ai_agent = PerceptionAIAgentNode()
    rclpy.spin(ai_agent)
    ai_agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()