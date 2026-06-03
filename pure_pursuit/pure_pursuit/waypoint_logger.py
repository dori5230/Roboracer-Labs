import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import csv 
import atexit

class WaypointLogger(Node):
    def __init__(self):
        super().__init__('waypoint_logger')

        self.filepath = '/sim_ws/src/pure_pursuit/pure_pursuit/path.csv'

        open(self.filepath, 'w').close()

        self.sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.save_waypoint, 10)



        """
        self.file = open('/sim_ws/src/pure_pursuit/path.csv', 'w', newline = '')
        self.writer = csv.writer(self.file)

        self.sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.save_waypoint, 10)

        atexit.register(self.shutdown)
        self.get_logger().info('Saving waypoints . . . ')
        """

    
    def save_waypoint(self, data):
        quaternion = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        ]

        _, _, yaw = euler_from_quaternion(quaternion)

        speed = np.linalg.norm([
            data.twist.twist.linear.x,
            data.twist.twist.linear.y,
            data.twist.twist.linear.z,
        ])

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        with open(self.filepath, 'a') as f:
            f.write(f'{x},{y},{yaw},{speed}\n')

        #self.writer.writerow([data.pose.pose.position.x, data.pose.pose.position.y, yaw, speed])

        #self.file.flush()
        self.get_logger().info(f'Recorded waypoint: {x:.2f}, {y:.2f}, yaw: {yaw:.2f}, speed{speed:.2f}')
    
    def shutdown(self):
        self.file.close()
        self.get_logger().info('Waypoints saved.')

    
def main(args = None):
    rclpy.init(args = args)
    node = WaypointLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()