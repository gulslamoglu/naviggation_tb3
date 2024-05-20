#! /usr/bin/env python3
#! /usr/bin/env python3
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
            Point(x=3.094, y=-0.195, z=0.0)
        ]
        self.index = 0
        self.timer = None
        self.start_publishing_with_delay()

    def start_publishing_with_delay(self):
        self.get_logger().info('Waiting for 10 seconds before starting to publish points...')
        time.sleep(10)
        self.get_logger().info('Starting to publish points.')
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_points)
    
    def publish_points(self):
        if self.index < len(self.points):
            point = self.points[self.index]
            self.publisher_.publish(point)
            self.get_logger().info(f'Publishing Point: [{point.x}, {point.y}, {point.z}]')
            self.index += 1
        else:
            self.get_logger().info('All initial points have been published.')
            self.timer.cancel()
            self.publish_random_points()

    def publish_random_points(self):
        time.sleep(3)
        self.get_logger().info('Publishing random points...')
        for _ in range(3):  # Publish 3 random points
            random_point = Point(
                x=random.uniform(2, 4),
                y=random.uniform(-2, 0),
                z=0.0
            )
            self.publisher_.publish(random_point)
            self.get_logger().info(f'Publishing Random Point: [{random_point.x}, {random_point.y}, {random_point.z}]')
            time.sleep(1)  # Small delay between publishing random points

def main(args=None):
    rclpy.init(args=args)
    point_publisher = PointPublisher()
    rclpy.spin(point_publisher)
    point_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
