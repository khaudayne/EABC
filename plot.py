import matplotlib.pyplot as plt
import numpy as np
from read_map import read_map_from_file, read_map_from_polygons
from convert_map import image_to_obstacle_map
def plot_map(route, obstacles, size_x = 500, size_y = 500):
    """
    Vẽ bản đồ với các vật cản (đa giác) trong không gian 500x500.
    :param obstacles: Danh sách các đa giác (danh sách các đỉnh) đại diện cho vật cản.
    :param size: Kích thước bản đồ.
    """
    fig, ax = plt.subplots()
    ax.set_xlim(0, size_x)
    ax.set_ylim(0, size_y)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_frame_on(True)
    for obstacle in obstacles:
        x, y = obstacle.exterior.xy
        ax.fill(x, y, edgecolor='black', color='black')

    for i in range(1, len(route) - 1):
        ax.scatter(route[i][0], route[i][1], color="#011F82", zorder=5, s=15)

    # Vẽ path của các con robot
    if len(route) > 1:
        tmp_arr = np.array(route)
        ax.plot(tmp_arr[:,0], tmp_arr[:,1], 'r--', color="#011F82", markersize=2)
        ax.plot(route[0][0], route[0][1], 'bo', markersize=8, label="Start")
        ax.plot(route[len(route) - 1][0], route[len(route) - 1][1], 'ro', markersize=8, label="Goal")
        ax.legend()
    plt.show()

### Cách đọc map .txt
# path_data = "data/map1.txt"
# map_size, obstacles, tree = read_map_from_file(path_data)
# plot_map([], obstacles)


### Cách đọc raw map
# map_size, polygons = image_to_obstacle_map("raw_picture/5.png")
# obstacles, tree = read_map_from_polygons(polygons)
# plot_map([], obstacles)

def plot_EABCDE(route1, route2, obj1, obj2, obstacles, loc_legend = 'upper left', size_x = 500, size_y = 500):
    """
    Vẽ bản đồ với các vật cản (đa giác) trong không gian 500x500.
    :param obstacles: Danh sách các đa giác (danh sách các đỉnh) đại diện cho vật cản.
    :param size: Kích thước bản đồ.
    """
    fig, ax = plt.subplots()
    ax.set_xlim(0, size_x)
    ax.set_ylim(0, size_y)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_frame_on(True)
    for obstacle in obstacles:
        x, y = obstacle.exterior.xy
        ax.fill(x, y, edgecolor='black', color='black')

    x1, y1 = zip(*route1)
    x2, y2 = zip(*route2)
    
    # Vẽ đường đi với marker và label chứa thông tin chiều dài
    plt.plot(x1, y1, color='green', marker='o', label=f'best length\npath length: {round(obj1[0], 2)}   path safety: {round(obj1[1], 2)}')
    plt.plot(x2, y2, color='orange', marker='s', label=f'best length\npath length: {round(obj2[0], 2)}   path safety: {round(obj2[1], 2)}')

    ax.plot(route1[0][0], route1[0][1], 'bo', markersize=8)
    ax.plot(route1[len(route1) - 1][0], route1[len(route1) - 1][1], 'ro', markersize=8)
    # Hiển thị
    plt.legend(loc=loc_legend)
    plt.show()
