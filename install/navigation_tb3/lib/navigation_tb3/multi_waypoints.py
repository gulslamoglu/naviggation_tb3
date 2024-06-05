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
        self.initial_pose = None  # initial_pose niteliği burada tanımlanıyor
        self.all_points = []
        self.visited_points = []  # Ziyaret edilen noktaları saklamak için

        # Engel konumları ve eşik mesafesi
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

    # Noktanın engellere çok yakın olup olmadığını kontrol eder
    def is_point_too_close_to_obstacle(self, point):
        for obstacle in self.obstacles:
            if self.calculate_distance(point[0], point[1], obstacle[0], obstacle[1]) < self.threshold_distance:
                return True
        return False

    # İki nokta arasındaki mesafeyi hesaplar
    def calculate_distance(self, x1, y1, x2, y2):
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    # Abonelikten gelen mesajı işler
    def listener_callback(self, msg):
        self.get_logger().info(f'Alınan Nokta: [{msg.x}, {msg.y}, {msg.z}]')

        if self.is_point_too_close_to_obstacle((msg.x, msg.y)):
            self.get_logger().info(f'Nokta [{msg.x}, {msg.y}] engele çok yakın. Bu nokta atlanıyor.')
            return

        self.security_route.append([msg.x, msg.y])
        self.all_points.append([msg.x, msg.y])
        
        if len(self.security_route) == 7:
            self.update_route_and_navigate_initial()

    # İlk 7 nokta için rota güncellenir ve robot hedefe yönlendirilir
    def update_route_and_navigate_initial(self):
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
            self.navigate_through_poses(route_poses, initial=True)

    # Pozlar üzerinden navigasyon gerçekleştirilir
    def navigate_through_poses(self, poses, initial=False):
        if len(poses) == 0:
            if initial:
                self.is_navigating = False
                self.security_route = []
                self.subscription.callback = self.listener_callback_for_random_points
                self.get_logger().info('İlk rota tamamlandı. Rastgele noktalar bekleniyor.')
            else:
                self.draw_final_graph()
                self.draw_visited_points_path()  # Ziyaret edilen noktaları çizdir
            return

        self.navigator.goToPose(poses[0])

        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Mevcut hedefin tamamlanma süresi tahmini: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' saniye.')

                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                    self.get_logger().info('Navigasyon 180 saniye sınırını aştı, istek iptal ediliyor.')
                    self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Hedefe ulaşıldı!')
            self.visited_points.append([poses[0].pose.position.x, poses[0].pose.position.y])  # Ziyaret edilen noktayı ekle
            self.security_route.pop(0)
            self.navigate_through_poses(poses[1:], initial)
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigasyon iptal edildi, çıkılıyor.')
            exit(1)
        elif result == TaskResult.FAILED:
            self.get_logger().info('Navigasyon başarısız oldu, bir sonraki hedefe tekrar yönlendiriliyor.')
            self.visited_points.append([poses[0].pose.position.x, poses[0].pose.position.y])  # Ziyaret edilen noktayı ekle
            self.security_route.pop(0)
            self.navigate_through_poses(poses[1:], initial)

    # Başlangıç noktası ve hedef noktalarla grafik oluşturulur
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

    # Grafiği çizer
    def draw_graph(self, graph):
        pos = nx.get_node_attributes(graph, 'pos')
        nx.draw(graph, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=10, font_weight='bold')
        labels = nx.get_edge_attributes(graph, 'weight')
        nx.draw_networkx_edge_labels(graph, pos, edge_labels=labels)
        plt.show()

    # Minimum yayılma ağacı ile rota planlanır
    def plan_path_with_mst(self, graph):
        mst = nx.minimum_spanning_tree(graph)
        return mst

    # Minimum yayılma ağacından düğümleri sırayla alır
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

    # Rastgele noktalar için mesaj dinleyicisi
    def listener_callback_for_random_points(self, msg):
        self.get_logger().info(f'Alınan Rastgele Nokta: [{msg.x}, {msg.y}, {msg.z}]')

        if self.is_point_too_close_to_obstacle((msg.x, msg.y)):
            self.get_logger().info(f'Nokta [{msg.x}, {msg.y}] engele çok yakın. Bu nokta atlanıyor.')
            return

        self.security_route.append([msg.x, msg.y])
        self.all_points.append([msg.x, msg.y])

        if len(self.security_route) >= 10:
            self.update_route_and_navigate_random()

    # Rastgele noktalar için rota güncellenir ve robot hedefe yönlendirilir
    def update_route_and_navigate_random(self):
        self.is_navigating = True
        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0

        if not self.security_route:
            self.get_logger().info('Navigasyon için nokta kalmadı.')
            self.is_navigating = False
            return

        graph = self.create_graph_with_initial_position(self.initial_pose, self.security_route)
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

    # Rastgele pozlar üzerinden navigasyon gerçekleştirilir
    def navigate_through_poses_random(self, poses):
        if len(poses) == 0:
            self.get_logger().info('Rastgele rota tamamlandı!')
            self.is_navigating = False
            self.draw_final_graph()  # En son grafiği çizdir
            self.draw_visited_points_path()  # Ziyaret edilen noktaları çizdir
            return

        self.navigator.goToPose(poses[0])

        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                self.get_logger().info('Mevcut hedefin tamamlanma süresi tahmini: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' saniye.')

                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                    self.get_logger().info('Navigasyon 180 saniye sınırını aştı, istek iptal ediliyor.')
                    self.navigator.cancelTask()

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Rastgele hedefe ulaşıldı!')
            self.visited_points.append([poses[0].pose.position.x, poses[0].pose.position.y])  # Ziyaret edilen noktayı ekle
            self.security_route.pop(0)
            self.navigate_through_poses_random(poses[1:])
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Rastgele navigasyon iptal edildi, çıkılıyor.')
            exit(1)
        elif result == TaskResult.FAILED:
            self.get_logger().info('Rastgele navigasyon başarısız oldu, bir sonraki hedefe tekrar yönlendiriliyor.')
            self.visited_points.append([poses[0].pose.position.x, poses[0].pose.position.y])  # Ziyaret edilen noktayı ekle
            self.security_route.pop(0)
            self.navigate_through_poses_random(poses[1:])

    # Son grafiği çizer
    def draw_final_graph(self):
        G = nx.Graph()
        start_x = self.initial_pose.pose.position.x
        start_y = self.initial_pose.pose.position.y
        G.add_node("Start", pos=(start_x, start_y))

        for i, pos in enumerate(self.all_points):
            G.add_node(f"Point {i+1}", pos=(pos[0], pos[1]))

        for i, pos in enumerate(self.all_points):
            distance = self.calculate_distance(start_x, start_y, pos[0], pos[1])
            G.add_edge("Start", f"Point {i+1}", weight=distance)

        for i in range(len(self.all_points)):
            for j in range(i+1, len(self.all_points)):
                distance = self.calculate_distance(self.all_points[i][0], self.all_points[i][1], self.all_points[j][0], self.all_points[j][1])
                if not G.has_edge(f"Point {i+1}", f"Point {j+1}"):
                    G.add_edge(f"Point {i+1}", f"Point {j+1}", weight=distance)

        self.draw_graph(G)

    # Ziyaret edilen noktaları sırayla çizer
    def draw_visited_points_path(self):
        G = nx.DiGraph()  # Yönlü grafik
        pos = {}
        
        start_x = self.initial_pose.pose.position.x
        start_y = self.initial_pose.pose.position.y
        pos["Start"] = (start_x, start_y)
        G.add_node("Start")

        for i, point in enumerate(self.visited_points):
            pos[f"Point {i+1}"] = (point[0], point[1])
            G.add_node(f"Point {i+1}")
            if i == 0:
                G.add_edge("Start", f"Point {i+1}")
            else:
                G.add_edge(f"Point {i}", f"Point {i+1}")

        nx.draw(G, pos, with_labels=True, node_size=700, node_color='skyblue', font_size=10, font_weight='bold')
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    point_subscriber = PointSubscriber()
    rclpy.spin(point_subscriber)
    point_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()