from shapely.geometry import Polygon, Point
from shapely.strtree import STRtree
def read_map_from_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    # Đọc kích thước bản đồ
    width, height = map(int, lines[0].split())
    
    # Đọc số lượng vật cản
    num_obstacles = int(lines[1])
    
    obstacles = []
    index = 2
    for _ in range(num_obstacles):
        num_vertices = int(lines[index])  # Số đỉnh của vật cản
        index += 1
        vertices = []
        for _ in range(num_vertices):
            x, y = map(int, lines[index].split())
            vertices.append((x, y))
            index += 1
        obstacles.append(vertices)
    
    for i in range(len(obstacles)):
        obstacles[i] = Polygon(obstacles[i])
    tree = STRtree(obstacles)
    return (width, height), obstacles, tree

def read_map_from_polygons(polygons):
    obstacles = []
    for poly in polygons:
        obstacles.append(Polygon(poly))

    tree = STRtree(obstacles)
    return obstacles, tree
