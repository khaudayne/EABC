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
