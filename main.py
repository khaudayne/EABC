##### Library
from read_map import read_map_from_file
from AStar import AStar
from space_segment import SegmentSpace
from RRT import RRT
from objective_solver import cal_objective, check_dominate
from plot import plot_map
import time

### Read info map
path_data = "data/map.txt"
map_size, obstacles, tree = read_map_from_file(path_data)
### END Read info map

### Param
p_s = 50 # Population size
start = (50, 50)
goal = (450, 450)
### END Param

### Hybrid initialization stratery
start_time = time.time()
POP = []
astar = AStar(start, goal, map_size, tree)
rrt = RRT(start, goal, map_size, tree)
space_segment = SegmentSpace(start, goal, 15, map_size, tree)
POP.append(astar.find_path())

for i in range(1, p_s):
    rrt.reset()
    S_n = rrt.find_path()
    S_m = space_segment.find_path()
    if S_n == None and S_m == None:
        print("Can't find any path at iterator: {}".format(i))
        continue
    obj_n = cal_objective(S_n, tree)
    obj_m = cal_objective(S_m, tree)
    
    # Check which solution dominate other one
    if check_dominate(obj_n, obj_m):
        POP.append(S_n)
    elif check_dominate(obj_m, obj_n):
        POP.append(S_m)
    else:
        if S_n[0] < S_m[0]:
            POP.append(S_n)
        else:
            POP.append(S_m)

end_time = time.time()
print("\nHybrid initialization stratery session end!\nTime process: {}\n".format(end_time - start_time))
### END Hybrid initialization stratery




