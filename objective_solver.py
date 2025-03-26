import math
from shapely.geometry import LineString

def cal_objective(path, tree):
    if path == None or len(path) == 0:
        return (math.inf, math.inf)
    total_dis = 0
    for i in range(len(path) - 1):
        total_dis = total_dis + math.sqrt((path[i + 1][0] - path[i][0]) ** 2 + (path[i + 1][1] - path[i][1]) ** 2)
    
    line_path = LineString([(p[0], p[1]) for p in path])
    idx_obs = tree.nearest(line_path)
    min_dis_obs = line_path.distance(tree.geometries.take(idx_obs))
    return (total_dis, -min_dis_obs)

### Function check if solution s2 is dominated by s1
def check_dominate(s1, s2):
    if len(s1) != len(s2):
        return False
    for i in range(len(s1)):
        if s1[i] > s2[i]:
            return False
    for i in range(len(s1)):
        if s1[i] < s2[i]:
            return True
    return False

def normalization(list_obj):
    if len(list_obj) == 0:
        return None
    sz = len(list_obj[0])
    min_obj = [math.inf] * sz
    max_obj = [-math.inf] * sz
    for i in range(len(list_obj)):
        for j in range(sz):
            min_obj[j] = min(min_obj[j], list_obj[i][j])
            max_obj[j] = max(max_obj[j], list_obj[i][j])
    nor = [[1 for _ in range(sz)] for i in range(len(list_obj))]
    is_boundary = [False for _ in range(len(list_obj))]
    for i in range(len(list_obj)):
        for j in range(sz):
            if list_obj[i][j] == min_obj[j]:
                is_boundary[i] = True
            if max_obj[j] == min_obj[j]:
                continue
            nor[i][j] = (list_obj[i][j] - min_obj[j]) / (max_obj[j] - min_obj[j]) 
            
    return nor, is_boundary
