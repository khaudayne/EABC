import random
from shapely.geometry import LineString
class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

### Class tìm đường đi bằng RRT
# Gồm các param:
    # start: tuple gồm tọa độ (x, y) của điểm bắt đầu
    # goal: tuple gồm tọa độ (x, y) của điểm kết thúc
    # map_size: tuple chứa thông tin kích cỡ của map (width, height)
    # str_tree: STRtree, gồm tree chứa bounding của các obstacle
    # step_size: bước nhảy tối đa khi tạo mới một điểm qua mỗi vòng lặp
    # max_iter: số vòng lặp tối đa để tạo ra node mới
    # goal_sample_rate: tỉ lệ lựa chọn hướng tới goal thay vì mở rộng ngẫu nhiên

# Tìm đường bằng hàm find_path():
    # Trả về đường nếu tìm thành không
    # Trả về None nếu không tìm được đường

class RRT:
    def __init__(self, start, goal, map_size, str_tree, step_size=50, max_iter=5000, goal_sample_rate=0.1):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_size = map_size
        self.str_tree = str_tree
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.tree = [self.start]

    def reset(self):
        self.tree = [self.start]

    def get_random_point(self):
        if random.random() < self.goal_sample_rate:
            return (self.goal.x, self.goal.y)
        return (random.randint(0, self.map_size[0]), random.randint(0, self.map_size[1]))

    def nearest_node(self, point):
        return min(self.tree, key=lambda node: (node.x - point[0])**2 + (node.y - point[1])**2)

    def is_collision_free(self, node1, node2):
        line = LineString([(node1.x, node1.y), (node2.x, node2.y)])
        candidates = self.str_tree.query(line)
        if len(candidates) > 0:
            return False
        return True

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
            if new_node == nearest:
                continue
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