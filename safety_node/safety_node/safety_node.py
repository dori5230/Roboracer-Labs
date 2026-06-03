import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.current_velocity = 0.0
        self.ttc_threshold = 0.6
        self.breaking = False

    def odom_callback(self, msg):
        self.current_velocity = msg.twist.twist.linear.x

    def scan_callback(self, msg):

        if self.current_velocity <= 0.0:
            return

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        range_rates = self.current_velocity *np.cos(angles)
        #front = np.abs(angles) < np.deg2ra d(40)

        approaching = range_rates > 1e-3
        ranges = ranges[approaching]
        range_rates = range_rates[approaching]

        if len(ranges) == 0:
            return

        iTTC = ranges/range_rates
            
        min_ttc = np.min(iTTC)

        if 0 < min_ttc < self.ttc_threshold:
            if not self.breaking:
                self.get_logger().warn(f"Brake! TTC: {min_ttc:.3f}")
            self.breaking = True
            self.publish_brake()
        else:
            self.breaking = False
        
        #self.get_logger().info(f"Min TTC: {min_ttc:.2f}")
        
    def publish_brake(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
