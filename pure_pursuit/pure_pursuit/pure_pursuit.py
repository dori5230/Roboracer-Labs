import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from tf_transformations import euler_from_quaternion

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        
        self.min_lookahead = 0.5
        self.max_lookahead = 2.0
        self.lookahead_ratio = 8.0
        self.curr_velocity = 0.0

        self.waypoints = None

        self.K_p = 0.5
        self.wheelbase = 0.33
        self.velocity_percentage = 1.0
        #self.safe_steering_angle = 0.418

        self.pose_sub = self.create_subscription(Odometry,
        '/ego_racecar/odom', self.pose_callback, 10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive', 10
        )

        self.waypoints = np.loadtxt ('/sim_ws/src/pure_pursuit/pure_pursuit/path.csv', delimiter = ',' , ndmin = 2)

    def pose_callback (self, msg):

        if self.waypoints is None:
            return
        
        self.curr_velocity = msg.twist.twist.linear.x

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        quaternion = msg.pose.pose.orientation
        orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

        _, _, yaw = euler_from_quaternion(orientation_list)


        distances = np.linalg.norm(self.waypoints[:, 0:2] - np.array([x,y]), axis = 1)
        closest_idx = np.argmin(distances)

        lookahead = np.clip(
            self.max_lookahead * self.curr_velocity / self.lookahead_ratio,
            self.min_lookahead,
            self.max_lookahead
        )

        lookahead_idx = closest_idx

        while lookahead_idx < len(self.waypoints):
            dx = self.waypoints[lookahead_idx, 0] - x 
            dy = self.waypoints[lookahead_idx, 1] - y
            if np.hypot (dx,dy) > lookahead:
                break
            lookahead_idx += 1

        if lookahead_idx >= len(self.waypoints):
            return

        
        goal = self.waypoints[lookahead_idx]
        goal_yaw = goal[2]
        goal_speed = goal[3]

        dx = goal[0] - x
        dy = goal[1] - y

        local_x = np.cos(-yaw) * dx - np.sin(-yaw) * dy
        local_y = np.sin(-yaw) * dx + np.cos(-yaw) * dy

        dyaw = goal_yaw - yaw
        dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))

        if local_x == 0: 
            return

        r = np.sqrt(local_x**2 + local_y**2)
        yaw_steering = 0.5 * dyaw
        steering_angle = (self.K_p * 2 * local_y/r**2) + yaw_steering
        steering_angle = np.clip(steering_angle, -0.418, 0.418)
        
        speed = goal_speed * self.velocity_percentage

        angle_deg = abs(np.degrees(steering_angle))
        if angle_deg >= 20:
            speed = min(speed, 2.0 * self.velocity_percentage)
        elif angle_deg >= 10:
            speed = min(speed, 2.5 * self.velocity_percentage)

        """
        curvature = (2*local_y)/ (self.lookahead_distance **2)
        steering_angle = np.arctan(self.wheelbase * curvature)
        steering_angle = np.clip(steering_angle, -0.418, 0.418)
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        

        self.drive_pub.publish(drive_msg)

def main (args = None):
    rclpy.init(args = args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
