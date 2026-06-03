import rclpy
from rclpy.node import Node
import numpy as np
import random
import math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

from tf_transformations import euler_from_quaternion

class TreeNode: 
    
    def __init__ (self, x, y):
        self.x = x 
        self.y = y
        self.parent = None

class RRT(Node): 
    def __init__(self):
        super().__init__('rrt_planner')


        self.expand_dist = 0.4
        self.goal_sample_rate = 10
        self.max_iter = 500
        self.search_radius = 4.0
        
        self.odom = None
        self.scan = None

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path, 
            '/rrt_path',
            10
        )

    def odom_callback(self, msg):
        self.odom = msg
    
    def scan_callback(self, msg):
        self.scan = msg
    



    def sample(self, start, goal):
            
        if random.randint(0,100) < self.goal_sample_rate:
            return TreeNode(goal[0], goal[1])

        x = random.uniform(
            start[0] - self.search_radius,
            start[0] + self.search_radius
        )
        y = random.uniform(
            start[1] - self.search_radius,
            start[1] + self.search_radius
        )

        return TreeNode(x,y)

    def nearest(self, node_list, rnd):
        distances = [
            (node.x - rnd.x)**2 + (node.y - rnd.y)**2
            for node in self.node_list
        ]

        return node_list[np.argmin(distances)]

    def steer(self, node1, node2):
        new = TreeNode(node1.x, node1.y)
        theta = math.atan2(node2.y - node1.y, node2.x - node1.x)

        new.x += self.expand_dist * math.cos(theta)
        new.y += self.expand_dist * math.sin(theta)

        new.parent = node1
            
        return new

    def collision (self, node):
        if self.scan is None:
            return False
        
        ranges = np.array(self.scan.ranges)
        ranges = ranges[np.isfinite(ranges)]

        if len(ranges) == 0:
            return True

        if np.min(ranges) < 0.2:
            return False
        
        return True
    
    def build_path (self, node):
        path = []

        while node is not None:
            path.append([node.x, node.y])
            node = node.parent
        
        return path[::-1]
    
    def publish_path (self, path):
        
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock.now().to_msg()

        for p in path:
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]

            msg.poses.append(pose)
        
        self.path_pub.publish(msg)


    def plan(self):

        if self.odom is None:
            return
        
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y

        q = self.odom.pose.pose.position.orientation
        orientation = [q.x, q.y, q.z, q.w]

        _,_,yaw = euler_from_quaternion(orientation)

        start = [x,y]

        goal = [
            x + 3*np.cos(yaw),
            y + 3*np.sin(yaw)
        ]

        start_node = TreeNode(start[0], start[1])
        node_list = [start_node]

        for i in range(self.max_iter):
            rnd = self.sample(start, goal)
            nearest = self.nearest(node_list, rnd)
            new_node = self.steer(nearest, rnd)

            if not self.collision(new_node):
                continue
            
            node_list.append(new_node)

            if math.hypot(new_node.x - goal[0], new_node.y - goal[1]) < self.expand_dist: 
                path = self.build_path(new_node)
                self.publish_path(path)
                return



        
def main (args = None):
    rclpy.init(args = args)
    node = RRT()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

