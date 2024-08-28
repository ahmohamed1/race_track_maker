#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import sys

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.declare_parameter('csv_file', 'path_points.csv')  # Declare a parameter with a default value
        self.publisher_ = self.create_publisher(Path, 'path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)  # Publish every second
        self.path = Path()
        self.path.header.frame_id = "map"
        csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        self.load_points_from_csv(csv_file)

    def load_points_from_csv(self, csv_file):
        with open(csv_file, 'r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(row['x'])
                pose.pose.position.y = float(row['y'])
                pose.pose.position.z = 0.0

                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 0.0

                self.path.poses.append(pose)

    def publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.path)
        self.get_logger().info('Publishing Path with %d points' % len(self.path.poses))


def main(args=None):
    rclpy.init()
    path_publisher = PathPublisher()
    rclpy.spin(path_publisher)
    path_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
