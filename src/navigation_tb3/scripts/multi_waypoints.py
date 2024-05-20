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
        self.is_navigating = False
        self.initial_pose = None

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Point: [{msg.x}, {msg.y}, {msg.z}]')
        self.security_route.append([msg.x, msg.y])
        if not self.is_navigating:
            self.start_security_route()

    def start_security_route(self):
        self.is_navigating = True
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.036
        self.initial_pose.pose.position.y = 0.009
        self.initial_pose.pose.orientation.z = -0.010
        self.initial_pose.pose.orientation.w = 0.99
        self.navigator.setInitialPose(self.initial_pose)
        self.update_route_and_navigate()

    def update_route_and_navigate(self):
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0

        graph = create_graph_with_initial_position(self.initial_pose, self.security_route)
        draw_graph(graph)
        path = plan_path_with_mst(graph)
        ordered_nodes = get_ordered_nodes_from_mst(path, "Start")

        for node in ordered_nodes:
            if node == "Start":
                continue
            trash_index = int(node.split()[1]) - 1
            pose.pose.position.x = self.security_route[trash_index][0]
            pose.pose.position.y = self.security_route[trash_index][1]
            route_poses.append(deepcopy(pose))

        self.navigate_through_poses(route_poses)

    def navigate_through_poses(self, poses):
        if len(poses) == 0:
            print('Route complete!')
            self.is_navigating = False
            return

        self.navigator.goToPose(poses[0])

        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time to complete current goal: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')

                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                    print('Navigation has exceeded timeout of 180s, canceling request.')
                    self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Reached goal!')
            self.navigate_through_poses(poses[1:])
        elif result == TaskResult.CANCELED:
            print('Navigation was canceled, exiting.')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Navigation failed, retrying next goal.')
            self.navigate_through_poses(poses[1:])

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
    return mst

def get_ordered_nodes_from_mst(mst, start_node):
    visited = set()
    ordered_nodes = []

    def dfs(node):
        visited.add(node)
        ordered_nodes.append(node)
        for neighbor in mst.neighbors(node):
            if neighbor not in visited:
                dfs(neighbor)

    dfs(start_node)
    return ordered_nodes

def main(args=None):
    rclpy.init(args=args)
    point_subscriber = PointSubscriber()
    rclpy.spin(point_subscriber)
    point_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
