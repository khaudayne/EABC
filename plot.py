import matplotlib.pyplot as plt
import numpy as np

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

    for i in range(len(route)):
        ax.scatter(route[i][0], route[i][1], color="#011F82", zorder=5, s=15)

    # Vẽ path của các con robot
    if len(route) > 1:
        tmp_arr = np.array(route)
        ax.plot(tmp_arr[:,0], tmp_arr[:,1], 'r--', color="#011F82", markersize=2)
    plt.show()