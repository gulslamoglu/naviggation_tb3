#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time
import random

class PointPublisher(Node):

    def __init__(self):
        super().__init__('point_publisher')
        self.publisher_ = self.create_publisher(Point, 'points', 10)
        self.points = [
            Point(x=2.298, y=-1.38, z=0.0),
            Point(x=3.794, y=-0.195, z=0.0),
            Point(x=3.094, y=-0.195, z=0.0),
            Point(x=1.5, y=0.5, z=0.0),
            Point(x=2.5, y=1.5, z=0.0),
            Point(x=3.5, y=2.5, z=0.0),
            Point(x=4.5, y=3.5, z=0.0)
        ]
        self.index = 0
        self.timer = None

        # Define obstacle points and threshold distance
        self.obstacles = [
            (3.09, 0.472),
            (3.06, 1.54),
            (3.08, -0.66),
            (1.97, -0.671),
            (2.02, 0.508),
            (2.01, 1.56),
            (0.876, 1.55),
            (0.84, 0.497),
            (0.882, -0.635)
        ]
        self.threshold_distance = 0.2

        self.start_publishing_with_delay()

    def is_point_too_close_to_obstacle(self, point):
        for obstacle in self.obstacles:
            if self.calculate_distance(point.x, point.y, obstacle[0], obstacle[1]) < self.threshold_distance:
                return True
        return False

    def calculate_distance(self, x1, y1, x2, y2):
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def start_publishing_with_delay(self):
        self.get_logger().info('Waiting for 10 seconds before starting to publish points...')
        time.sleep(10)
        self.get_logger().info('Starting to publish points.')
        self.timer = self.create_timer(1.0, self.publish_points)

    def publish_points(self):
        if self.index < len(self.points):
            point = self.points[self.index]
            if not self.is_point_too_close_to_obstacle(point):
                self.publisher_.publish(point)
                self.get_logger().info(f'Publishing Point: [{point.x}, {point.y}, {point.z}]')
            else:
                self.get_logger().info(f'Skipping Point [{point.x}, {point.y}] as it is too close to an obstacle.')
            self.index += 1
        else:
            self.get_logger().info('All initial points have been published.')
            self.timer.cancel()
            self.publish_random_points()

    def publish_random_points(self):
        count = 0
        while rclpy.ok() and count < 10:
            random_point = Point(
                x=random.uniform(2, 4),
                y=random.uniform(-2, 0),
                z=0.0
            )
            if not self.is_point_too_close_to_obstacle(random_point):
                self.publisher_.publish(random_point)
                self.get_logger().info(f'Publishing Random Point: [{random_point.x}, {random_point.y}, {random_point.z}]')
                count += 1
            else:
                self.get_logger().info(f'Skipping Random Point [{random_point.x}, {random_point.y}] as it is too close to an obstacle.')

def main(args=None):
    rclpy.init(args=args)
    point_publisher = PointPublisher()
    rclpy.spin(point_publisher)
    point_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
