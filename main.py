##### Library
from read_map import read_map_from_file
from AStar import AStar
from space_segment import SegmentSpace
from RRT import RRT
from objective_solver import cal_objective, check_dominate, normalization
from plot import plot_map
import time
import random
import math
from geometry import path_crossover_operator, path_mutation_operator, path_safety_operator, path_shortening_operator, fast_non_dominated_sort
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)
warnings.filterwarnings("ignore", category=UserWarning)

### Read info map
path_data = "data/map3.txt"
map_size, obstacles, tree = read_map_from_file(path_data)
### END Read info map

### Param
p_s = 50 # Population size
c_ef = 10 # Max count non-evolution individual to become scout bee
c_mf = 20
start = (50, 50)
goal = (50, 350)
MAX_CIRCLE = 1
TIME_LIMIT = 50
### END Param

print("START algorithm with param: ")
print("Path map: {}".format(path_data))
print("Population size p_s: {}".format(p_s))
print("Max count non-evolution individual to become scout bee c_ef: {}".format(c_ef))
print("Max time mutation path c_mf: {}".format(c_mf))
print("Position start: {}".format(start))
print("Position goal: {}".format(goal))
print("Max circle: {}".format(MAX_CIRCLE))
print("Time limit algorithm: {}".format(TIME_LIMIT))
print("#########################################################################################")
print("\nStart Hybrid initialization stratery!")
### Hybrid initialization stratery
start_time = time.time()
POP = []
stagnation_count = []
astar = AStar(start, goal, map_size, tree)
rrt = RRT(start, goal, map_size, tree, step_size=5, max_iter=10000)
space_segment = SegmentSpace(start, goal, 15, map_size, tree, number_try=25)
POP.append(astar.find_path())
stagnation_count.append(0)

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
    stagnation_count.append(0)

end_time = time.time()
print("\nHybrid initialization stratery session end!\nTime process: {}\n".format(end_time - start_time))
### END Hybrid initialization stratery

print("#########################################################################################")
print("\nStart EABC algorithm!")
### EABC algorithm
circle = 1
while circle <= MAX_CIRCLE and end_time - start_time <= TIME_LIMIT:
    if circle % 10 == 0:
        print("EABC iterator: {}".format(circle))

    ### Increase stagnation_count
    for i in range(len(stagnation_count)):
        stagnation_count[i] += 1

    ### Employed bee phase
    for i in range(len(POP)):
        S_i = POP[i]
        random_idx = random.randint(0, len(POP) - 1)
        while random_idx == i:
            random_idx = random.randint(0, len(POP) - 1)
        S_random = POP[random_idx]
        S_new = path_crossover_operator(S_i, S_random, tree)
        S_new = path_mutation_operator(S_new, tree, c_mf)

        if random.random() > 0.5:
            S_new = path_shortening_operator(S_new, tree)
        else:
            S_new = path_safety_operator(S_new, tree)
        
        if not(POP[i] is S_new):
            stagnation_count[i] = 0
        POP[i] = S_new


    NDS_archive_idx, POP_ns_idx, list_obj = fast_non_dominated_sort(POP, tree)
    ### Onlooker bee phase

    # Dominance-guided optimization mechanism
    for i in range(len(POP_ns_idx)):
        old_obj = list_obj[POP_ns_idx[i]]
        path = POP[POP_ns_idx[i]][:]
        path = path_shortening_operator(path, tree)
        path = path_safety_operator(path, tree)
        idx_random = random.randint(0, len(NDS_archive_idx) - 1)
        path_nds_random = POP[NDS_archive_idx[idx_random]][:]
        path = path_crossover_operator(path, path_nds_random, tree)

        new_obj = cal_objective(path, tree)
        if check_dominate(new_obj, old_obj):
            stagnation_count[POP_ns_idx[i]] = 0
            POP[POP_ns_idx[i]] = path

    # Collaborative-based optimization mechanism
    NDS_objective_value = [list_obj[nds_idx] for nds_idx in NDS_archive_idx]
    NDS_normalizee_value, is_boundary = normalization(NDS_objective_value)
    for i in range(len(NDS_archive_idx)):
        path = POP[NDS_archive_idx[i]][:]
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
            stagnation_count[NDS_archive_idx[i]] = 0
            POP[NDS_archive_idx[i]] = path

    NDS_archive_idx, POP_ns_idx, list_obj = fast_non_dominated_sort(POP, tree)
    NDS_objective_value = [list_obj[nds_idx] for nds_idx in NDS_archive_idx]
    NDS_normalizee_value, is_boundary = normalization(NDS_objective_value)
    list_idx_boundary = []
    for j in range(len(is_boundary)):
        if is_boundary[j]:
             list_idx_boundary.append(NDS_archive_idx[j])
    ### Scout bee phase
    for i in range(len(POP_ns_idx)):
        if stagnation_count[POP_ns_idx[i]] >= c_ef:
            print("Nhay vao dayyyyyy")
            if random.random() > 0.5:
                idx_nds_random = NDS_archive_idx[random.randint(0, len(NDS_archive_idx) - 1)]
                idx_nds_boundary_random = NDS_archive_idx[random.randint(0, len(NDS_archive_idx) - 1)]
                if len(list_idx_boundary) > 0:
                    idx_nds_boundary_random = list_idx_boundary[random.randint(0, len(list_idx_boundary) - 1)]
                nds_path = POP[idx_nds_random][:]
                nds_boundary = POP[idx_nds_boundary_random][:]
                POP[POP_ns_idx[i]] = path_crossover_operator(nds_boundary, nds_path, tree)
            else:
                number_RRT_path = 5
                final_rrt_path = None
                min_path_len = math.inf
                for j in range(number_RRT_path):
                    rrt.reset()
                    rrt_new_path = rrt.find_path()
                    obj_rrt_path = cal_objective(rrt_new_path, tree)
                    if min_path_len > obj_rrt_path[0]: # Compare path length
                        min_path_len = obj_rrt_path[0]
                        final_rrt_path = rrt_new_path
                POP[POP_ns_idx[i]] = final_rrt_path
                
            stagnation_count[POP_ns_idx[i]] = 0

    circle = circle + 1
    end_time = time.time()

print("\nEABC session end at iterator: {}, time process: {}\n".format(circle - 1, end_time - start_time))
### END EABC algorithm
print("#########################################################################################")
NDS_archive_idx, POP_ns_idx, list_obj = fast_non_dominated_sort(POP, tree)
print("\nEND algorithm, show result below:\n")
for i in range(len(NDS_archive_idx)):
    path = POP[NDS_archive_idx[i]]
    print("\nRount {}: {}".format(i + 1, path))
    obj = list_obj[NDS_archive_idx[i]]
    print("Have objective value is: {}".format(obj))
plot_map(POP[NDS_archive_idx[random.randint(0, len(NDS_archive_idx) - 1)]], obstacles)



