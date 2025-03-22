import random
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, LineString
from read_map import read_map_from_file
import time
class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRT:
    def __init__(self, start, goal, map_size, obstacles, step_size=100, max_iter=5000, goal_sample_rate=0.2):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_size = map_size
        self.obstacles = [Polygon(obstacle) for obstacle in obstacles]
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.tree = [self.start]

    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return (self.goal.x, self.goal.y)
        return (random.randint(0, self.map_size[0]), random.randint(0, self.map_size[1]))

    def nearest_node(self, point):
        return min(self.tree, key=lambda node: (node.x - point[0])**2 + (node.y - point[1])**2)

    def is_collision_free(self, node1, node2):
        line = LineString([(node1.x, node1.y), (node2.x, node2.y)])
        return all(not line.intersects(obstacle) for obstacle in self.obstacles)

    def steer(self, from_node, to_point):
        direction_x, direction_y = to_point[0] - from_node.x, to_point[1] - from_node.y
        length = (direction_x**2 + direction_y**2) ** 0.5
        
        if length == 0:
            return from_node  # Tránh chia cho 0
        
        scale = min(self.step_size / length, 1)
        new_x, new_y = from_node.x + scale * direction_x, from_node.y + scale * direction_y
        return Node(int(new_x), int(new_y), from_node)

    def find_path(self):
        for _ in range(self.max_iter):
            rand_point = self.get_random_point()
            nearest = self.nearest_node(rand_point)
            new_node = self.steer(nearest, rand_point)
            
            if self.is_collision_free(nearest, new_node):
                self.tree.append(new_node)
                
                if ((new_node.x - self.goal.x) ** 2 + (new_node.y - self.goal.y) ** 2) <= self.step_size ** 2 and self.is_collision_free(new_node, self.goal):
                    self.goal.parent = new_node
                    return self.get_path()
        return None

    def get_path(self):
        path = []
        node = self.goal
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]

    def plot(self, path=None):
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.map_size[0])
        ax.set_ylim(0, self.map_size[1])
        
        for obstacle in self.obstacles:
            x, y = obstacle.exterior.xy
            ax.fill(x, y, color='black')
        
        for node in self.tree:
            if node.parent:
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'g-', alpha=0.5)
        
        if path:
            path_x, path_y = zip(*path)
            ax.plot(path_x, path_y, 'r-', linewidth=2)
        
        ax.plot(self.start.x, self.start.y, 'bo', markersize=8, label="Start")
        ax.plot(self.goal.x, self.goal.y, 'ro', markersize=8, label="Goal")
        ax.legend()
        plt.show()

# Đọc dữ liệu từ file
file_path = "data/map.txt"
map_size, obstacles = read_map_from_file(file_path)
start = (50, 50)
goal = (450, 450)

# Chạy thuật toán RRT
start_time = time.time()
rrt = RRT(start, goal, map_size, obstacles)
path = rrt.find_path()
end_time = time.time()
print("TIME RUN PROCESS: {}".format(end_time - start_time))
print(path)
rrt.plot(path)