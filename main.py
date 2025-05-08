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
import numpy as np
from shapely.geometry import LineString
from geometry import path_crossover_operator_new, path_crossover_operator, path_mutation_operator, path_safety_operator, path_shortening_operator, fast_non_dominated_sort
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)
warnings.filterwarnings("ignore", category=UserWarning)
from moo_algorithm.nsga_ii import run_nsga_ii
from moo_algorithm.nsga_iii import run_nsga_iii
from moo_algorithm.mode import run_mode
from moo_algorithm.mopso import run_mopso
from moo_algorithm.moead import run_moead, init_weight_vectors_2d
from population import Population, Individual
from moo_algorithm.metric import cal_hv, cal_igd

### Read info map
path_data = "data/map3.txt"
map_size, obstacles, tree = read_map_from_file(path_data)
### END Read info map

### Param
p_s = 50 # Population size
c_ef = 10 # Max count non-evolution individual to become scout bee
c_mf = 20
start = (39, 20)
goal = (364, 455)
MAX_CIRCLE = 30
TIME_LIMIT = 50
REF_POINT = [-math.inf, -math.inf]
### END Param

# print("START algorithm with param: ")
# print("Path map: {}".format(path_data))
# print("Population size p_s: {}".format(p_s))
# print("Max count non-evolution individual to become scout bee c_ef: {}".format(c_ef))
# print("Max time mutation path c_mf: {}".format(c_mf))
# print("Position start: {}".format(start))
# print("Position goal: {}".format(goal))
# print("Max circle: {}".format(MAX_CIRCLE))
# print("Time limit algorithm: {}".format(TIME_LIMIT))
# print("#########################################################################################")
# print("\nStart Hybrid initialization stratery!")
### Hybrid initialization stratery
POP = []
stagnation_count = []
astar = AStar(start, goal, map_size, tree)
rrt = RRT(start, goal, map_size, tree, step_size=15, max_iter=10000)
space_segment = SegmentSpace(start, goal, 15, map_size, tree, number_try=25)

start_time = time.time()
print("EABC")
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
# print("\nHybrid initialization stratery session end!\nTime process: {}\n".format(end_time - start_time))
### END Hybrid initialization stratery

# print("#########################################################################################")
### EABC algorithm
circle = 1
while circle <= MAX_CIRCLE and end_time - start_time <= TIME_LIMIT:
    # if circle % 10 == 0:
        # print("EABC iterator: {}".format(circle))

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
            # print("Nhay vao dayyyyyy")
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

# print("\nEABC session end at iterator: {}, time process: {}\n".format(circle - 1, end_time - start_time))

### END EABC algorithm
# print("#########################################################################################")
NDS_archive_idx, POP_ns_idx, list_obj = fast_non_dominated_sort(POP, tree)
# print("\nEND algorithm, show result below:\n")
# for i in range(len(NDS_archive_idx)):
#     path = POP[NDS_archive_idx[i]]
#     print("\nRount {}: {}".format(i + 1, path))
#     obj = list_obj[NDS_archive_idx[i]]
#     print("Have objective value is: {}".format(obj))
# plot_map(POP[NDS_archive_idx[random.randint(0, len(NDS_archive_idx) - 1)]], obstacles)

EABC_log = [list_obj[i] for i in NDS_archive_idx]
for obj in EABC_log:
    REF_POINT[0] = max(REF_POINT[0], obj[0])
    REF_POINT[1] = max(REF_POINT[1], obj[1])

end_time = time.time()
print("EABC Done!")
print("Time run: {}".format(end_time - start_time))

## Run NSGA ii
pop_size = 50
indi_list = []
max_gen = 100
start_time = time.time()
for i in range(pop_size):
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
        indi_list.append(Individual(S_n))
    elif check_dominate(obj_m, obj_n):
        indi_list.append(Individual(S_m))
    else:
        if S_n[0] < S_m[0]:
            indi_list.append(Individual(S_n))
        else:
            indi_list.append(Individual(S_m))

NSGA_ii_log = run_nsga_ii(tree, obstacles, indi_list, pop_size, max_gen, path_crossover_operator, path_mutation_operator, 0.5, 0.1, cal_objective)
for obj in NSGA_ii_log:
    REF_POINT[0] = max(REF_POINT[0], obj[0])
    REF_POINT[1] = max(REF_POINT[1], obj[1])
end_time = time.time()
print("Time run: {}".format(end_time - start_time))

## Run NSGA iii
indi_list = []
start_time = time.time()
for i in range(pop_size):
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
        indi_list.append(Individual(S_n))
    elif check_dominate(obj_m, obj_n):
        indi_list.append(Individual(S_m))
    else:
        if S_n[0] < S_m[0]:
            indi_list.append(Individual(S_n))
        else:
            indi_list.append(Individual(S_m))

NSGA_iii_log = run_nsga_iii(tree, obstacles, indi_list, pop_size, max_gen, path_crossover_operator, path_mutation_operator, 0.5, 0.1, cal_objective)
for obj in NSGA_iii_log:
    REF_POINT[0] = max(REF_POINT[0], obj[0])
    REF_POINT[1] = max(REF_POINT[1], obj[1])
end_time = time.time()
print("Time run: {}".format(end_time - start_time))

## Run MODE
indi_list = []
start_time = time.time()
for i in range(pop_size):
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
        indi_list.append(Individual(S_n))
    elif check_dominate(obj_m, obj_n):
        indi_list.append(Individual(S_m))
    else:
        if S_n[0] < S_m[0]:
            indi_list.append(Individual(S_n))
        else:
            indi_list.append(Individual(S_m))

MODE_log = run_mode(tree, obstacles, indi_list, pop_size, max_gen, 0.5, 0.9, cal_objective)
for obj in MODE_log:
    REF_POINT[0] = max(REF_POINT[0], obj[0])
    REF_POINT[1] = max(REF_POINT[1], obj[1])
end_time = time.time()
print("Time run: {}".format(end_time - start_time))

## Run MOPSO
indi_list = []
start_time = time.time()
for i in range(pop_size):
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
        indi_list.append(Individual(S_n))
    elif check_dominate(obj_m, obj_n):
        indi_list.append(Individual(S_m))
    else:
        if S_n[0] < S_m[0]:
            indi_list.append(Individual(S_n))
        else:
            indi_list.append(Individual(S_m))

MOPSO_log = run_mopso(tree, obstacles, indi_list, pop_size, max_gen, 0.9, 0.4, 1.5, 1.5, cal_objective)
for obj in MOPSO_log:
    REF_POINT[0] = max(REF_POINT[0], obj[0])
    REF_POINT[1] = max(REF_POINT[1], obj[1])
end_time = time.time()
print("Time run: {}".format(end_time - start_time))

## Run MOEAD
indi_list = []
start_time = time.time()
for i in range(pop_size):
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
        indi_list.append(Individual(S_n))
    elif check_dominate(obj_m, obj_n):
        indi_list.append(Individual(S_m))
    else:
        if S_n[0] < S_m[0]:
            indi_list.append(Individual(S_n))
        else:
            indi_list.append(Individual(S_m))

MOEAD_log = run_moead(tree, obstacles, indi_list, pop_size, max_gen, 5, init_weight_vectors_2d, path_crossover_operator, path_mutation_operator, cal_objective)
for obj in MOEAD_log:
    REF_POINT[0] = max(REF_POINT[0], obj[0])
    REF_POINT[1] = max(REF_POINT[1], obj[1])
end_time = time.time()
print("Time run: {}".format(end_time - start_time))

# Run EABCDE
p_s = 50 # Population size
c_ef = 3 # Max count non-evolution individual to become scout bee
c_mf = 20
p_mutation = 0.2
MAX_CIRCLE = 100
TIME_LIMIT = 15
POP = []
stagnation_count = []
start_time = time.time()
print("EABCDE")
for i in range(p_s):
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
circle = 1
while circle <= MAX_CIRCLE and end_time - start_time <= TIME_LIMIT:
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
        S_new = path_crossover_operator_new(S_i, S_random, tree)
        if random.random() < p_mutation:
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
        path_nds_random = POP[NDS_archive_idx[idx_random]]
        path = path_crossover_operator_new(path, path_nds_random, tree)
        
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

    ### Scout bee phase
    list_idx_scout = []
    for i in range(len(stagnation_count)):
        if(stagnation_count[i] >= c_ef):
            list_idx_scout.append(i)

    if len(list_idx_scout) > 0:
        # gen_offspring_de
        offspring = []
        
        for i in range(len(POP)):
            indi = POP[i]
            indi1 = POP[random.randint(0, len(POP) - 1)]
            indi2 = POP[random.randint(0, len(POP) - 1)]

            # rand_1
            l1 = []
            l2 = []
            for p in indi:
                min_dis = -1
                idx = -1
                for i in range(len(indi1)):
                    p1 = indi1[i]
                    if min_dis == -1 or min_dis > (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2:
                        min_dis = (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2
                        idx = i
                l1.append(indi1[idx])

                min_dis = -1
                idx = -1
                for i in range(len(indi2)):
                    p1 = indi2[i]
                    if min_dis == -1 or min_dis > (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2:
                        min_dis = (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2
                        idx = i
                l2.append(indi2[idx])
            new_indi = []
            for i, p in enumerate(indi):
                if random.random() > 0.9:
                    new_indi.append(p)
                else:
                    new_indi.append((int(p[0] + 0.5 * (l1[i][0] - l2[i][0])), int(p[1] + 0.5 * (l1[i][1] - l2[i][1]))))
            line = LineString([p for p in new_indi])
            candidates = tree.query(line, predicate='intersects')
            if len(candidates) == 0:
                offspring.append(new_indi)
        
        if len(offspring) <= len(list_idx_scout):
            for i in range(len(offspring)):
                POP[list_idx_scout[i]] = offspring[i]
                stagnation_count[list_idx_scout[i]] = 0
        else:
            sz_offspring = len(offspring)
            crowding_distance = [0] * sz_offspring
            offspring.extend(POP)
            NDS_archive_idx, _, list_obj = fast_non_dominated_sort(offspring, tree)
            new_list_obj = [
                [list_obj[idx], idx] for idx in NDS_archive_idx
            ]

            # calcculate crowding distance
            for m in range(2):
                new_list_obj.sort(key=lambda x: x[0][m])
                m_values = [x[0][m] for x in new_list_obj]
                scale = max(m_values) - min(m_values)
                if scale == 0: scale = 1

                for idx in NDS_archive_idx:
                    if idx < sz_offspring:
                        tmp_idx = None
                        for i in range(len(new_list_obj)):
                            if idx == new_list_obj[i][1]:
                                tmp_idx = i
                                break
                        if tmp_idx == 0 or tmp_idx == len(new_list_obj) - 1:
                            crowding_distance[idx] = 100000000
                        else:
                            crowding_distance[idx] += (new_list_obj[i + 1][0][m] - new_list_obj[i - 1][0][m]) / scale
                    else:
                        break
            list_idx_offstring = np.argsort(crowding_distance)[-len(list_idx_scout):][::-1].tolist()
            for i in range(len(list_idx_scout)):
                POP[list_idx_scout[i]] = offspring[list_idx_offstring[i]]
                stagnation_count[list_idx_scout[i]] = 0


    circle = circle + 1
    end_time = time.time()

for i in range(len(POP)):
    indi = POP[i]
    indi1 = POP[random.randint(0, len(POP) - 1)]
    indi2 = POP[random.randint(0, len(POP) - 1)]
     # rand_1
    l1 = []
    l2 = []
    for p in indi:
        min_dis = -1
        idx = -1
        for i in range(len(indi1)):
            p1 = indi1[i]
            if min_dis == -1 or min_dis > (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2:
                min_dis = (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2
                idx = i
        l1.append(indi1[idx])
        min_dis = -1
        idx = -1
        for i in range(len(indi2)):
            p1 = indi2[i]
            if min_dis == -1 or min_dis > (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2:
                min_dis = (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2
                idx = i
        l2.append(indi2[idx])
    new_indi = []
    for i, p in enumerate(indi):
        if random.random() > 0.9:
            new_indi.append(p)
        else:
            new_indi.append((int(p[0] + 0.5 * (l1[i][0] - l2[i][0])), int(p[1] + 0.5 * (l1[i][1] - l2[i][1]))))
    line = LineString([p for p in new_indi])
    candidates = tree.query(line, predicate='intersects')
    if len(candidates) == 0:
        POP.append(new_indi)

NDS_archive_idx, POP_ns_idx, list_obj = fast_non_dominated_sort(POP, tree)
EABCDE_log = [list_obj[i] for i in NDS_archive_idx]
for obj in EABCDE_log:
    REF_POINT[0] = max(REF_POINT[0], obj[0])
    REF_POINT[1] = max(REF_POINT[1], obj[1])

end_time = time.time()
print("EABCDE Done!")
print("Time run: {}".format(end_time - start_time))


print("REF POINT: {}".format(REF_POINT))
EABC_hv = cal_hv(EABC_log, REF_POINT)
print("EABC HV: {}".format(EABC_hv))
NSGA_ii_hv = cal_hv(NSGA_ii_log, REF_POINT)
print("NSGA ii HV: {}".format(NSGA_ii_hv))
NSGA_iii_hv = cal_hv(NSGA_iii_log, REF_POINT)
print("NSGA iii HV: {}".format(NSGA_iii_hv))
MODE_hv = cal_hv(MODE_log, REF_POINT)
print("MODE HV: {}".format(MODE_hv))
MOPSO_hv = cal_hv(MOPSO_log, REF_POINT)
print("MOPSO HV: {}".format(MOPSO_hv))
MOEAD_hv = cal_hv(MOEAD_log, REF_POINT)
print("MOEAD HV: {}".format(MOEAD_hv))
EABCDE_hv = cal_hv(EABCDE_log, REF_POINT)
print("EABCDE HV: {}".format(EABCDE_hv))
# with open('log_metric/log_hv.txt', 'a') as f:
#     f.write("{} {} {} {} {} {}\n".format(EABC_hv, NSGA_ii_hv, NSGA_iii_hv, MODE_hv, MOPSO_hv, MOEAD_hv))






