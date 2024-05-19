#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from copy import deepcopy
import networkx as nx
import matplotlib.pyplot as plt

class PointSubscriber(Node):

    def __init__(self):
        super().__init__('point_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'points',
            self.listener_callback,
            10)
        self.subscription
        self.security_route = []
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Point: [{msg.x}, {msg.y}, {msg.z}]')
        self.security_route.append([msg.x, msg.y])
        if len(self.security_route) == 3:  # or any condition you prefer
            self.start_security_route()

    def start_security_route(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.036
        initial_pose.pose.position.y = 0.009
        initial_pose.pose.orientation.z = -0.010
        initial_pose.pose.orientation.w = 0.99
        self.navigator.setInitialPose(initial_pose)

        while rclpy.ok():
            route_poses = []
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.orientation.w = 1.0

            graph = create_graph_with_initial_position(initial_pose, self.security_route)
            draw_graph(graph)
            path = plan_path_with_mst(graph)

            for edge in path:
                for node in edge:
                    if node == "Start":
                        continue
                    trash_index = int(node.split()[1]) - 1
                    pose.pose.position.x = self.security_route[trash_index][0]
                    pose.pose.position.y = self.security_route[trash_index][1]
                    route_poses.append(deepcopy(pose))

            self.navigator.goThroughPoses(route_poses)

            i = 0
            while not self.navigator.isTaskComplete():
                i += 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time to complete current route: ' + '{0:.0f}'.format(
                          Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                          + ' seconds.')

                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                        print('Navigation has exceeded timeout of 180s, canceling request.')
                        self.navigator.cancelTask()

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Route complete! Restarting...')
            elif result == TaskResult.CANCELED:
                print('Security route was canceled, exiting.')
                exit(1)
            elif result == TaskResult.FAILED:
                print('Security route failed! Restarting from other side...')

def create_graph_with_initial_position(initial_pose, trash_positions):
    G = nx.Graph()
    
    start_x = initial_pose.pose.position.x 
    start_y = initial_pose.pose.position.y
    G.add_node("Start", pos=(start_x, start_y))

    for i, pos in enumerate(trash_positions):
        G.add_node(f"Trash {i+1}", pos=(pos[0], pos[1]))
    
    for i, pos in enumerate(trash_positions):
        distance = calculate_distance(start_x, start_y, pos[0], pos[1])
        G.add_edge("Start", f"Trash {i+1}", weight=distance)

    for i in range(len(trash_positions)):
        for j in range(i+1, len(trash_positions)):
            distance = calculate_distance(trash_positions[i][0], trash_positions[i][1], trash_positions[j][0], trash_positions[j][1])
            if not G.has_edge(f"Trash {i+1}", f"Trash {j+1}"):
                G.add_edge(f"Trash {i+1}", f"Trash {j+1}", weight=distance)

    return G

def calculate_distance(x1, y1, x2, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

def draw_graph(graph):
    pos = nx.get_node_attributes(graph, 'pos')
    nx.draw(graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=10, font_weight='bold')
    labels = nx.get_edge_attributes(graph, 'weight')
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=labels)
    plt.show()

def plan_path_with_mst(graph):
    mst = nx.minimum_spanning_tree(graph)
    paths = list(mst.edges)
    return paths

def main(args=None):
    rclpy.init(args=args)
    point_subscriber = PointSubscriber()
    rclpy.spin(point_subscriber)
    point_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
