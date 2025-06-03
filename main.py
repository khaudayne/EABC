##### Library
from read_map import read_map_from_polygons
from convert_map import image_to_obstacle_map
from space_segment import SegmentSpace
from RRT import RRT
from objective_solver import cal_objective, check_dominate, normalization
from plot import plot_EABCDE
import time
import random
import numpy as np
from shapely.geometry import LineString
from geometry import path_crossover_operator_new, path_mutation_operator, path_safety_operator, path_shortening_operator, fast_non_dominated_sort, fnds
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)
warnings.filterwarnings("ignore", category=UserWarning)
from moo_algorithm.mode import run_mode
from population import Individual

### Define test case
list_start_test_case = [
    [(120, 60), (36, 453), (23, 202), (201, 26)],
    [(36, 101), (32, 439), (28, 350), (225, 93)],
    [(39, 20), (44, 485), (9, 320), (297, 21)]
]
list_end_test_case = [
    [(454, 437), (469, 121), (477, 339), (170, 472)],
    [(429, 473), (486, 164), (487, 272), (251, 471)],
    [(364, 455), (461, 32), (421, 265), (271, 487)]
]
### Read info map
# path_data = "data/map3.txt"
# map_size, obstacles, tree = read_map_from_file(path_data)

map_size, polygons = image_to_obstacle_map("raw_picture/10.png")
obstacles, tree = read_map_from_polygons(polygons)
### END Read info map

start = (404, 429)
goal = (407, 28)
print("START POINT: {}, GOAL POINT: {}".format(start, goal))
# Run EABCDE
p_s = 50 # Population size
c_ef = 2 # Max count non-evolution individual to become scout bee
c_mf = 20
p_mutation = 0.2
MAX_CIRCLE = 100
TIME_LIMIT = 15
POP = []
stagnation_count = []
rrt = RRT(start, goal, map_size, tree, step_size=15, max_iter=10000)
space_segment = SegmentSpace(start, goal, 15, map_size, tree, number_try=25)


start_time = time.time()
print("\nEABCDE")
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

tmp_start_time = time.time()
end_time = time.time()

circle = 1
while circle <= MAX_CIRCLE and end_time - tmp_start_time <= TIME_LIMIT:
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

indi_list = []
for path in POP:
    indi_list.append(Individual(path))
POP = run_mode(tree, obstacles, indi_list, len(indi_list), 50, 0.5, 0.9, cal_objective)
NDS_archive_idx, POP_ns_idx, list_obj = fast_non_dominated_sort(POP, tree)

min_length = 10000000
idx_min_length = -1
min_safety = 10000000
idx_min_safety = -1

print("\nEND algorithm, show result below:\n")
for i in range(len(NDS_archive_idx)):
    obj = list_obj[NDS_archive_idx[i]]
    if min_length > obj[0]:
        min_length = obj[0]
        idx_min_length = NDS_archive_idx[i]
    
    if min_safety > obj[1]:
        min_safety = obj[1]
        idx_min_safety = NDS_archive_idx[i]
plot_EABCDE(POP[idx_min_length], POP[idx_min_safety], list_obj[idx_min_length], list_obj[idx_min_safety], obstacles, 'upper left')