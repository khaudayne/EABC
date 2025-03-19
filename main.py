from bee import initial_hive
import random
import numpy as np

def read_distance_matrix(filename = "data/distance_matrix.txt"):
    with open(filename, "r") as f:
        lines = f.readlines()
    n = int(lines[0].strip())
    distance_matrix = [list(map(int, line.split())) for line in lines[1:n+1]]
    return n, distance_matrix

number_city, distance_matrix = read_distance_matrix()

# Param
TOTAL_BEE = 10
EMPLOYED_BEE_PERCENT = 0.5
CIRCLE_LIMIT = 50
MAX_CIRCLE = 500

circle = 1
hive, min_dis, path_res, number_employed_bee, number_onlooker_bee = initial_hive(TOTAL_BEE, EMPLOYED_BEE_PERCENT, number_city, distance_matrix)

while circle <= MAX_CIRCLE:
    new_population = []
    fitness = []
    # Phase của employed bee
    for i in range(number_employed_bee):
        bee = hive[i]
        if bee.role == 'E':
            k = random.randint(0, number_employed_bee)
            while k == i:
                k = random.randint(0, number_employed_bee)
            tmp_dis, tmp_path = bee.modified_path(list(hive[k].path), distance_matrix, CIRCLE_LIMIT)
            new_population.append(tmp_path)
            fitness.append(tmp_dis)
            if min_dis > tmp_dis:
                min_dis = tmp_dis
                path_res = list(tmp_path)
    
    # Tính phân phối lựa chọn cho onlooker bee
    min_fitness = min(fitness)
    fitness_shifted = np.array(fitness) - min_fitness
    max_fitness_shifted = np.max(fitness_shifted)
    probabilities = None
    if max_fitness_shifted == 0:
        probabilities = np.ones(len(fitness)) / len(fitness)  
    else:
        probabilities = np.exp(-fitness_shifted / (max_fitness_shifted + 1e-10))
        probabilities /= probabilities.sum()

    # Phase của onlooker bee
    for i in range(number_onlooker_bee):
        bee = hive[i + number_employed_bee]
        if bee.role == 'O':
            # Onlooker bee sẽ có xác suất tìm tới path của các employed bee có độ fitness cao (distance thấp)
            idx_employed_bee = np.random.choice(len(new_population), p=probabilities)
            bee.path = list(hive[idx_employed_bee].path)
            bee.distance = hive[idx_employed_bee].distance
            k = random.randint(0, number_employed_bee)
            while k == idx_employed_bee:
                k = random.randint(0, number_employed_bee)
            tmp_dis, tmp_path = bee.modified_path(list(hive[k].path), distance_matrix, CIRCLE_LIMIT)
            if min_dis > tmp_dis:
                min_dis = tmp_dis
                path_res = list(tmp_path)

    # Phase của scout bee
    for i in range(number_employed_bee):
        bee = hive[i]
        if bee.role == 'S':
            tmp_dis, tmp_path = bee.shuffer_path(distance_matrix)
            if min_dis > tmp_dis:
                min_dis = tmp_dis
                path_res = list(tmp_path)
    
    if circle % 100 == 0:
        print("\tProcess iterator: {}".format(circle))
    circle += 1

print("Process end after {} iterator with result: ".format(circle - 1))
print("Min distance between cities: {}".format(min_dis))
print("Route is: {}".format(path_res))




