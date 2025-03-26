##### Library
from read_map import read_map_from_file
from AStar import AStar
from space_segment import SegmentSpace
from RRT import RRT
from objective_solver import cal_objective, check_dominate, normalization
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
    for i in range(len(POP)):
        S_i = POP[i][:]
        random_idx = random.randint(0, len(POP) - 1)
        while random_idx == i:
            random_idx = random.randint(0, len(POP) - 1)
        S_random = POP[random_idx][:]
        S_new = path_crossover_operator(S_i, S_random, tree)
        S_new = path_mutation_operator(S_new, tree)

        if random.random() > 0.5:
            POP[i] = path_shortening_operator(S_new, tree)
        else:
            POP[i] = path_safety_operator(S_new, tree)


    NDS_archive, POP_ns = fast_non_dominated_sort(POP, tree)
    ### Onlooker bee phase

    # Collaborative-based optimization mechanism
    NDS_objective_value = [cal_objective(nds, tree) for nds in NDS_archive]
    NDS_normalizee_value, is_boundary = normalization(NDS_objective_value)
    for i in range(len(NDS_archive)):
        path = NDS_archive[i][:]
        if is_boundary[i]:
            path = path_shortening_operator(path, tree)
            path = path_safety_operator(path, tree)
        else:
            if NDS_normalizee_value[i][0] > NDS_normalizee_value[i][1]: # Compare normalize between length path and safety of path
                path = path_shortening_operator(path, tree)
            else:
                path = path_safety_operator(path, tree)
        
        new_obj = cal_objective(path, tree)
        if check_dominate(new_obj, NDS_objective_value[i]):
            NDS_archive[i] = path

    # Dominance-guided optimization mechanism
    for i in range(len(POP_ns)):
        old_obj = cal_objective(POP_ns[i], tree)
        path = POP_ns[i][:]
        path = path_shortening_operator(path, tree)
        path = path_safety_operator(path, tree)
        idx_random = random.randint(0, len(NDS_archive) - 1)
        path_nds_random = NDS_archive[idx_random][:]
        path = path_crossover_operator(path, path_nds_random, tree)

        new_obj = cal_objective(path, tree)
        if check_dominate(new_obj, old_obj):
            POP_ns[i] = path

    POP = NDS_archive + POP_ns

    ### Scout bee phase
    

    circle = circle + 1
    end_time = time.time()

end_time = time.time()
print("\nEABC session end at iterator: {}, time process: {}\n".format(circle - 1, end_time - start_time))
### END EABC algorithm

NDS_archive, POP_ns = fast_non_dominated_sort(POP, tree)
plot_map(NDS_archive[random.randint(0, len(NDS_archive) - 1)], obstacles)



