import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollow (Node):

    def __init__(self):
        super().__init__('wall_follow')

        self.K_p = 1.0
        self.K_i = 0.0
        self.K_d = 0.1

        self.desired_distance = 1.0

        self.L = 0.5

        self.theta = 50

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def get_range(self, scan_msg, angle_deg):
        """
        Get the laser scan range at a given angle in degrees.
        Angles are relative to the car's x-axis (front).
        Negative = right, positive = left.
        """
        angle_rad = np.radians(angle_deg)
        index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
        index = np.clip(index, 0, len(scan_msg.ranges) - 1)
        distance = scan_msg.ranges[index]
 
        # Handle inf/nan
        if np.isinf(distance) or np.isnan(distance):
            distance = scan_msg.range_max
 
        return distance
 
    def get_error(self, scan_msg):
        """
        Calculate the error between desired and estimated future distance to wall.
        Following the left wall (left turn = counter-clockwise).
        """
        theta = self.theta
 
        # Beam b: 90 degrees to the left of car's x-axis
        b = self.get_range(scan_msg, 90.0)
 
        # Beam a: theta degrees from beam b (towards front-right)
        a = self.get_range(scan_msg, 90.0 - theta)
 
        # Calculate alpha
        alpha = np.arctan2(
            a * np.cos(np.radians(theta)) - b,
            a * np.sin(np.radians(theta))
        )
 
        # Current distance to wall
        D_t = b * np.cos(alpha)
 
        # Future distance to wall
        D_t1 = D_t + self.L * np.sin(alpha)
 
        error = self.desired_distance - D_t1
 
        return error
 
    def pid_control(self, error):
        """
        Calculate steering angle using PID control.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
 
        if dt == 0:
            dt = 0.01
 
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
 
        steering_angle = (
            self.K_p * error +
            self.K_i * self.integral +
            self.K_d * derivative
        )
 
        # Clamp steering angle
        steering_angle = np.clip(-steering_angle, -0.418, 0.418)
 
        self.prev_error = error
        self.prev_time = current_time
 
        return steering_angle
 
    def scan_callback(self, scan_msg):
        error = self.get_error(scan_msg)
        steering_angle = self.pid_control(error)
 
        # Speed based on steering angle
        angle_deg = abs(np.degrees(steering_angle))
        if angle_deg <= 10.0:
            speed = 1.5
        elif angle_deg <= 20.0:
            speed = 1.0
        else:
            speed = 0.5
 
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
 
        self.drive_pub.publish(drive_msg)
 
        self.get_logger().info(
            f'Error: {error:.3f} | Steering: {np.degrees(steering_angle):.2f} deg | Speed: {speed:.1f} m/s'
        )
 
 
def main(args=None):
    rclpy.init(args=args)
    node = WallFollow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()

