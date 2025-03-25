##### Library
from read_map import read_map_from_file
from AStar import AStar
from space_segment import SegmentSpace
from RRT import RRT
from objective_solver import cal_objective, check_dominate
from plot import plot_map
import time
import random
from geometry import path_crossover_operator, path_mutation_operator, path_safety_operator, path_shortening_operator, fast_non_dominated_sort
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)
warnings.filterwarnings("ignore", category=UserWarning)

### Read info map
path_data = "data/map.txt"
map_size, obstacles, tree = read_map_from_file(path_data)
### END Read info map

### Param
p_s = 50 # Population size
start = (50, 50)
goal = (450, 450)
MAX_CIRCLE = 100
TIME_LIMIT = 100
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

### EABC algorithm
circle = 1
start_time = time.time()
end_time = start_time
while circle <= MAX_CIRCLE and end_time - start_time <= TIME_LIMIT:
    if circle % 10 == 0:
        print("EABC iterator: {}".format(circle))

    ### Employed bee phase
    for i in range(p_s):
        S_i = POP[i][:]
        random_idx = random.randint(0, p_s - 1)
        while random_idx == i:
            random_idx = random.randint(0, p_s - 1)
        S_random = POP[random_idx][:]
        S_new = path_crossover_operator(S_i, S_random, tree)
        S_new = path_mutation_operator(S_new, tree)

        if random.random() > 0.5:
            POP[i] = path_shortening_operator(S_new, tree)
        else:
            POP[i] = path_safety_operator(S_new, tree)



    ### Onlooker bee phase


    ### Scout bee phase

    circle = circle + 1
    end_time = time.time()

end_time = time.time()
print("\nEABC session end at iterator: {}, time process: {}\n".format(circle - 1, end_time - start_time))
### END EABC algorithm

NDS_archive, POP_ns = fast_non_dominated_sort(POP, tree)
plot_map(NDS_archive[random.randint(0, len(NDS_archive) - 1)], obstacles)



