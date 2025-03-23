from shapely.geometry import Point, LineString, Polygon
from shapely.strtree import STRtree
from shapely.ops import nearest_points

def is_point_in_obstacle(point, obstacles):
    """
    Kiểm tra xem một điểm có nằm trên hoặc trong một vật cản hay không.
    :param point: Tuple (x, y) đại diện cho điểm cần kiểm tra.
    :param obstacles: Danh sách các đa giác (danh sách các đỉnh) đại diện cho vật cản.
    :return: True nếu điểm nằm trên hoặc trong vật cản, ngược lại False.
    """
    point_geom = Point(point)
    
    for obstacle in obstacles:
        if obstacle.contains(point_geom) or obstacle.touches(point_geom):
            return True
    
    return False

def does_line_intersect_obstacle(point1, point2, obstacles):
    """
    Kiểm tra xem một đoạn thẳng có cắt qua bất kỳ vật cản nào không.
    :param point1: Tuple (x, y) đại diện điểm đầu.
    :param point2: Tuple (x, y) đại diện điểm cuối.
    :param obstacles: Danh sách các đa giác (danh sách các đỉnh) đại diện cho vật cản.
    :return: True nếu đoạn thẳng cắt qua vật cản, ngược lại False.
    """
    line = LineString([point1, point2])
    
    for obstacle in obstacles:
        if line.intersects(obstacle):
            return True
    
    return False

def min_distance_point_to_line(point, point1, point2):
    """
    Tìm khoảng cách nhỏ nhất từ một điểm đến đoạn thẳng nối giữa hai điểm.
    :param point: Tuple (x, y) đại diện cho điểm cần đo khoảng cách.
    :param point1: Tuple (x, y) đại diện điểm đầu của đoạn thẳng.
    :param point2: Tuple (x, y) đại diện điểm cuối của đoạn thẳng.
    :return: Khoảng cách nhỏ nhất từ điểm đến đoạn thẳng.
    """
    line = LineString([point1, point2])
    point_geom = Point(point)
    return point_geom.distance(line)

obstacles = [
    Polygon([(1, 1), (4, 1), (4, 4), (1, 4)]),  # Hình vuông (1,1) -> (4,4)
    Polygon([(5, 5), (8, 5), (8, 8), (5, 8)]),  # Hình vuông (5,5) -> (8,8)
    Polygon([(6, 1), (9, 1), (9, 3), (6, 3)])   # Hình chữ nhật (6,1) -> (9,3)
]

# Tạo đoạn thẳng
line = LineString([(0, 0), (1, 10)])

# Tạo đa giác
polygon = Polygon([(2, 2), (2, 4), (4, 4), (4, 2)])

# Tìm điểm gần nhất giữa đoạn thẳng và đa giác
point_on_line, point_on_polygon = nearest_points(line, polygon)

# Kết quả
print("Điểm trên đoạn thẳng gần nhất với đa giác:", point_on_line)
print("Điểm trên đa giác gần nhất với đoạn thẳng:", point_on_polygon)