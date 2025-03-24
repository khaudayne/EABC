from shapely.geometry import LineString
import math
import random

### Class tìm đường đi bằng space segmentation method
# Gồm các param:
    # start: tuple gồm tọa độ (x, y) của điểm bắt đầu
    # goal: tuple gồm tọa độ (x, y) của điểm kết thúc
    # num_segments: số lượng đoạn chia trong trục OX'
    # map_size: tuple chứa thông tin kích cỡ của map (width, height)
    # tree: STRtree, gồm tree chứa bounding của các obstacle
    # number_try: số lần tạo ngẫu nhiên các điểm trên mỗi segment để tạo thành đường dẫn cho robot
    # delta_y: dựa theo kích cỡ của map, để xác định tọa độ ngẫu nhiên của các điểm tạo mới nằm trong khoảng nào

# Tìm đường bằng hàm find_path():
    # Trả về đường nếu tìm thành không
    # Trả về None nếu không tìm được đường

class SegmentSpace:
    def __init__(self, start, goal, num_segments, map_size, tree, number_try = 100, delta_y = 100):
        self.start = start
        self.goal = goal
        self.num_segments = num_segments
        self.map_size = map_size
        self.tree = tree

        arc_tan = math.atan((goal[1] - start[1]) / (goal[0] - start[0]))
        self.sin_phi = math.sin(arc_tan)
        self.cos_phi = math.cos(arc_tan)

        self.number_try = number_try
        self.delta_y = delta_y

        dis = math.sqrt((goal[1] - start[1]) ** 2 + (goal[0] - start[0]) ** 2)
        self.delta_x = dis / (num_segments + 1)

    def is_valid(self, old_point, new_point, is_last_point):
        x, y, _ = new_point
        if not (0 <= x <= self.map_size[0] and 0 <= y <= self.map_size[1]):
            return False
        line = LineString([(old_point[0], old_point[1]), (new_point[0], new_point[1])])
        is_collision = len(self.tree.query(line)) > 0
        if is_collision:
            return False
        if not is_last_point:
            return True
        else:
            line = LineString([(new_point[0], new_point[1]), (self.goal[0], self.goal[1])])
            return len(self.tree.query(line)) == 0
    
    def convert_to_global_cord(self, x, y):
        global_x = round(self.start[0] + x * self.cos_phi - y * self.sin_phi)
        global_y = round(self.start[1] + x * self.sin_phi + y * self.cos_phi)
        return [global_x, global_y, -1]


    def find_path(self):
        path = [[self.start]]
        for i in range(1, self.num_segments + 1):
            check_try = False
            for _ in range(self.number_try):
                local_y = random.uniform(-self.delta_y, self.delta_y)
                local_x = i * self.delta_x
                next_point = self.convert_to_global_cord(local_x, local_y)
                for k in range(len(path[i - 1])):
                    if self.is_valid(path[i - 1][k], next_point, i == self.num_segments):
                        if len(path) <= i:
                            path.append([])

                        next_point[2] = k
                        path[i].append(next_point)
                        check_try = True
                        break
                
            if not check_try:
                return None
        path.append([self.goal])
        return self.generate_random_path(path)

    def generate_random_path(self, path):
        res = []
        i = len(path)
        res.append((path[i - 1][0][0], path[i - 1][0][1]))
        i = i - 2
        point = path[i][random.randint(0, len(path[i]) - 1)]
        while i > 0:
            res.append((point[0], point[1]))
            i = i - 1
            point = path[i][point[2]]
        res.append((path[0][0][0], path[0][0][1]))
        return res[::-1]


# map_size, obstacles = read_map_from_file("data/map.txt")
# # Thông số thuật toán
# start = (50, 50)
# goal = (450, 450)
# num_segments = 30

# # Khởi tạo quần thể đường đi
# path_init = SegmentSpace(start, goal, num_segments, map_size, STRtree([Polygon(obs) for obs in obstacles]), 20, 100)
# population = path_init.find_path()
# print(population)
