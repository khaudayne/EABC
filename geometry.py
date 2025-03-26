from objective_solver import cal_objective, check_dominate
from shapely.strtree import STRtree
from shapely.geometry import Point, LineString
from shapely.ops import nearest_points
import math
import random

def path_crossover_operator(path1, path2, tree):
    if len(path1) <= 2 or len(path2) <= 2: # Không có intermediate point
        return path1
    
    len1 = len(path1)
    len2 = len(path2)

    tmp_tree = STRtree([Point(p[0], p[1]) for p in path1[1:len1 - 1]])
    min_dis = math.inf
    idx1 = -1
    idx2 = -1
    
    for i in range(1, len2 - 1):
        p = path2[i]
        point = Point(p[0], p[1])
        idx_point = tmp_tree.nearest(point)
        dis = point.distance(tmp_tree.geometries.take(idx_point))
        if min_dis > dis:
            min_dis = dis
            idx1 = idx_point
            idx2 = i
    
    idx1 = idx1 + 1 # Do lấy các intermediate point có idx từ [1:len - 2] thành [0:len - 3] trong tmp_tree nên cần + 1
    new_path1 = None
    new_path2 = None
    if path1[idx1][0] == path2[idx2][0] and path1[idx1][1] == path2[idx2][1]: # Khong lay cac diem trung nhau
        new_path1 = path1[:idx1 + 1] + path2[idx2 + 1:]
        new_path2 = path2[:idx2 + 1] + path1[idx1 + 1:]
    else:
        if len(tree.query(LineString([path1[idx1], path2[idx2]]))) > 0:
            return path1
        new_path1 = path1[:idx1 + 1] + path2[idx2:]
        new_path2 = path2[:idx2 + 1] + path1[idx1:]
    
    list_path = [path1, new_path1, new_path2]
    NDS_archive_idx, POP_ns_idx, _ = fast_non_dominated_sort(list_path, tree)
    sz = len(NDS_archive_idx)
    return list_path[NDS_archive_idx[random.randint(0, sz - 1)]]
    

def path_mutation_operator(path, tree, c_mf = 20):
    sz = len(path)
    if sz <= 2:
        return path
    objective_old = cal_objective(path, tree)
    count = 0
    while count < c_mf:
        idx_ran = random.randint(1, sz - 2)
        p_rand = path[idx_ran]
        point = Point(p_rand[0], p_rand[1])
        dis = point.distance(tree.geometries.take(tree.nearest(point)))
        radian_random = random.uniform(0, 2 * math.pi)
        p_new = (round(p_rand[0] + 2 * dis * math.cos(radian_random)), round(p_rand[1] + 2 * dis * math.sin(radian_random)))
        line = LineString([path[idx_ran - 1], p_new, path[idx_ran + 1]])
        if len(tree.query(line)) == 0:
            new_path = path[:idx_ran] + [p_new] + path[idx_ran + 1:]
            objective_new = cal_objective(new_path, tree)
            if check_dominate(objective_new, objective_old):
                return new_path

        count += 1
    return path

def path_shortening_operator(path, tree):
    n = len(path)
    if n <= 2: 
        return path
    count = 0
    while count < n * 2:
        i = random.randint(0, n - 1)
        j = random.randint(0, n - 1)
        if i > j:
            i, j = j, i
        if j - i > 1:
            line = LineString([path[i], path[j]])
            if len(tree.query(line)) == 0:
                new_path = path[:i + 1] + path[j:]
                return new_path
        count += 1
    return path

def path_safety_operator(path, tree):
    number_segment = len(path) - 1
    idx_segment_min = -1
    idx_obstacle_min = -1
    min_dis = math.inf
    for i in range(number_segment):
        line = LineString([path[i], path[i + 1]])
        idx_obs = tree.nearest(line)
        dis = line.distance(tree.geometries.take(idx_obs))
        if min_dis > dis:
            min_dis = dis
            idx_obstacle_min = idx_obs
            idx_segment_min = i
    line = LineString([path[idx_segment_min], path[idx_segment_min + 1]])
    point_on_line, point_on_polygon = nearest_points(line, tree.geometries.take(idx_obstacle_min))
    new_point = (round(point_on_line.x * 2 - point_on_polygon.x), round(point_on_line.y * 2 - point_on_polygon.y))
    if new_point[0] == path[idx_segment_min][0] and new_point[1] == path[idx_segment_min][1]:
        return path
    if new_point[0] == path[idx_segment_min + 1][0] and new_point[1] == path[idx_segment_min + 1][1]:
        return path
    line = LineString([path[idx_segment_min], new_point, path[idx_segment_min + 1]])
    if len(tree.query(line)) == 0 and idx_segment_min >= 0:
        new_path = path[:idx_segment_min + 1] + [new_point] + path[idx_segment_min + 1:]
        return new_path
    return path


# todo: toi uu cai nay
def fast_non_dominated_sort(paths, tree):
    values1 = []
    values2 = []
    list_obj = []
    for path in paths:
        obj_value = cal_objective(path, tree)
        list_obj.append(obj_value)
        values1.append(obj_value[0])
        values2.append(obj_value[1])

    S = [[] for _ in range(len(values1))]
    front = [[]]
    n = [0 for _ in range(len(values1))]
    rank = [0 for _ in range(len(values1))]

    for p in range(len(values1)):
        S[p] = []
        n[p] = 0
        for q in range(len(values1)):
            if (values1[p] < values1[q] and values2[p] < values2[q]) or \
               (values1[p] <= values1[q] and values2[p] < values2[q]) or \
               (values1[p] < values1[q] and values2[p] <= values2[q]):
                S[p].append(q)
            elif (values1[q] < values1[p] and values2[q] < values2[p]) or \
                 (values1[q] <= values1[p] and values2[q] < values2[p]) or \
                 (values1[q] < values1[p] and values2[q] <= values2[p]):
                n[p] += 1
        if n[p] == 0:
            rank[p] = 0
            if p not in front[0]:
                front[0].append(p)

    i = 0
    while front[i]:
        Q = []
        for p in front[i]:
            for q in S[p]:
                n[q] -= 1
                if n[q] == 0:
                    rank[q] = i + 1
                    if q not in Q:
                        Q.append(q)
        i += 1
        front.append(Q)
    del front[-1]
    NDS_archive_idx = []
    POP_ns_idx = []
    for i in range(len(front[0])):
        NDS_archive_idx.append(front[0][i])
    for i in range(1, len(front)):
        for j in range(len(front[i])):
            POP_ns_idx.append(front[i][j])
    
    return NDS_archive_idx, POP_ns_idx, list_obj
