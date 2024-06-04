#!/usr/bin/env python3
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
        self.security_route = []
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.is_navigating = False
        self.current_goal = None
        self.initial_pose = None  # Initialize initial_pose attribute here

        # Define obstacle locations and threshold distance
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

    def is_point_too_close_to_obstacle(self, point):
        for obstacle in self.obstacles:
            if self.calculate_distance(point[0], point[1], obstacle[0], obstacle[1]) < self.threshold_distance:
                return True
        return False

    def calculate_distance(self, x1, y1, x2, y2):
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Point: [{msg.x}, {msg.y}, {msg.z}]')

        if self.is_point_too_close_to_obstacle((msg.x, msg.y)):
            self.get_logger().info(f'Point [{msg.x}, {msg.y}] is too close to an obstacle. Skipping this point.')
            return

        self.security_route.append([msg.x, msg.y])
        if len(self.security_route) == 7:
            self.update_route_and_navigate()

    def update_route_and_navigate(self):
        self.is_navigating = True
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0

        if self.initial_pose is None:
            self.initial_pose = PoseStamped()
            self.initial_pose.header.frame_id = 'map'
            self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            self.initial_pose.pose.position.x = 0.036
            self.initial_pose.pose.position.y = 0.009
            self.initial_pose.pose.orientation.z = -0.010
            self.initial_pose.pose.orientation.w = 0.99
            self.navigator.setInitialPose(self.initial_pose)

        graph = self.create_graph_with_initial_position(self.initial_pose, self.security_route)
        self.draw_graph(graph)
        path = self.plan_path_with_mst(graph)
        ordered_nodes = self.get_ordered_nodes_from_mst(path, "Start")

        for node in ordered_nodes:
            if node == "Start":
                continue
            trash_index = int(node.split()[1]) - 1
            pose.pose.position.x = self.security_route[trash_index][0]
            pose.pose.position.y = self.security_route[trash_index][1]
            route_poses.append(deepcopy(pose))

        if route_poses:
            self.current_goal = route_poses[0]
            self.navigate_through_poses(route_poses)

    def navigate_through_poses(self, poses):
        if len(poses) == 0:
            print('Route complete!')
            self.is_navigating = False
            self.security_route = []
            self.subscription.callback = self.listener_callback_for_random_points
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
            # Remove the visited point from security_route
            self.security_route.pop(0)
            self.navigate_through_poses(poses[1:])
        elif result == TaskResult.CANCELED:
            print('Navigation was canceled, exiting.')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Navigation failed, retrying next goal.')
            # Remove the visited point from security_route even if it failed
            self.security_route.pop(0)
            self.navigate_through_poses(poses[1:])

    def create_graph_with_initial_position(self, initial_pose, trash_positions):
        G = nx.Graph()
        
        start_x = initial_pose.pose.position.x 
        start_y = initial_pose.pose.position.y
        G.add_node("Start", pos=(start_x, start_y))

        for i, pos in enumerate(trash_positions):
            G.add_node(f"Trash {i+1}", pos=(pos[0], pos[1]))
        
        for i, pos in enumerate(trash_positions):
            distance = self.calculate_distance(start_x, start_y, pos[0], pos[1])
            G.add_edge("Start", f"Trash {i+1}", weight=distance)

        for i in range(len(trash_positions)):
            for j in range(i+1, len(trash_positions)):
                distance = self.calculate_distance(trash_positions[i][0], trash_positions[i][1], trash_positions[j][0], trash_positions[j][1])
                if not G.has_edge(f"Trash {i+1}", f"Trash {j+1}"):
                    G.add_edge(f"Trash {i+1}", f"Trash {j+1}", weight=distance)

        return G

    def calculate_distance(self, x1, y1, x2, y2):
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def draw_graph(self, graph):
        pos = nx.get_node_attributes(graph, 'pos')
        nx.draw(graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=10, font_weight='bold')
        labels = nx.get_edge_attributes(graph, 'weight')
        nx.draw_networkx_edge_labels(graph, pos, edge_labels=labels)
        plt.show()

    def plan_path_with_mst(self, graph):
        mst = nx.minimum_spanning_tree(graph)
        return mst

    def get_ordered_nodes_from_mst(self, mst, start_node):
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

    def listener_callback_for_random_points(self, msg):
        self.get_logger().info(f'Received Random Point: [{msg.x}, {msg.y}, {msg.z}]')

        if self.is_point_too_close_to_obstacle((msg.x, msg.y)):
            self.get_logger().info(f'Random Point [{msg.x}, {msg.y}] is too close to an obstacle. Skipping this point.')
            return

        self.security_route.append([msg.x, msg.y])
        if len(self.security_route) == 1:
            self.update_route_and_navigate_random()

    def update_route_and_navigate_random(self):
        self.is_navigating = True
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0

        if not self.security_route:
            print('No more points to navigate.')
            self.is_navigating = False
            return

        graph = self.create_graph_with_initial_position(self.initial_pose, self.security_route)
        self.draw_graph(graph)
        path = self.plan_path_with_mst(graph)
        ordered_nodes = self.get_ordered_nodes_from_mst(path, "Start")

        for node in ordered_nodes:
            if node == "Start":
                continue
            trash_index = int(node.split()[1]) - 1
            pose.pose.position.x = self.security_route[trash_index][0]
            pose.pose.position.y = self.security_route[trash_index][1]
            route_poses.append(deepcopy(pose))

        if route_poses:
            self.current_goal = route_poses[0]
            self.navigate_through_poses_random(route_poses)

    def navigate_through_poses_random(self, poses):
        if len(poses) == 0:
            print('Random route complete!')
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
            print('Reached random goal!')
            self.security_route.pop(0)
            self.navigate_through_poses_random(poses[1:])
        elif result == TaskResult.CANCELED:
            print('Random navigation was canceled, exiting.')
            exit(1)
        elif result == TaskResult.FAILED:
            print('Random navigation failed, retrying next goal.')
            self.security_route.pop(0)
            self.navigate_through_poses_random(poses[1:])

def main(args=None):
    rclpy.init(args=args)
    point_subscriber = PointSubscriber()
    rclpy.spin(point_subscriber)
    point_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
