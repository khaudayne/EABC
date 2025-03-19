def total_distance(route, distance_matrix):
    dist = sum(distance_matrix[route[i]][route[i + 1]] for i in range(len(route) - 1))
    dist += distance_matrix[route[-1]][route[0]]
    return dist