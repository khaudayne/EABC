import math
import random
from distance import total_distance
class Bee:
    def __init__(self, path):
        self.role = ''
        self.path = list(path)
        self.distance = 0
        self.cycle = 0 
    
    def modified_path(self, other_path, distance_matrix, CIRCLE_LIMIT):
        ### Thực hiện các phép biến đổi đường đi:
        memory_path = list(self.path)

        is_greater = False
        # Đột biến:
        tmp_path_mutate = list(memory_path)
        a, b = random.sample(range(len(tmp_path_mutate)), 2)
        tmp_path_mutate[a], tmp_path_mutate[b] = tmp_path_mutate[b], tmp_path_mutate[a]
        tmp_dis = total_distance(tmp_path_mutate, distance_matrix)
        if self.distance > tmp_dis:
            self.distance = tmp_dis
            self.path = tmp_path_mutate
            is_greater = True
        
        # 2-opt
        a, b = sorted(random.sample(range(len(memory_path)), 2))
        tmp_path_two_opt = memory_path[ : a] + memory_path[a : b + 1][::-1] + memory_path[b + 1 :]
        tmp_dis = total_distance(tmp_path_two_opt, distance_matrix)
        if self.distance > tmp_dis:
            self.distance = tmp_dis
            self.path = tmp_path_two_opt
            is_greater = True

        # crossover 2 path
        size = len(memory_path)
        a, b = sorted(random.sample(range(size), 2))
        tmp_path_cross_over = [-1] * size
        tmp_path_cross_over[a:b+1] = memory_path[a:b+1]
        pos = b + 1
        for city in other_path:
            if city not in tmp_path_cross_over:
                if pos >= size:
                    pos = 0
                tmp_path_cross_over[pos] = city
                pos += 1
        tmp_dis = total_distance(tmp_path_cross_over, distance_matrix)
        if self.distance > tmp_dis:
            self.distance = tmp_dis
            self.path = tmp_path_cross_over
            is_greater = True

        if (not is_greater) and self.role == 'E':
            self.cycle += 1
            if self.cycle >= CIRCLE_LIMIT:
                self.role = 'S'
                self.cycle = 0
        return self.distance, self.path
    
    def shuffer_path(self, distance_matrix): 
        random.shuffle(self.path)
        self.cycle = 0
        self.role = 'E'
        self.distance = total_distance(self.path, distance_matrix)
        return self.distance, self.path

def initial_hive(number_bee, employed_bee_per, number_city, distance_matrix):
    # Khởi tạo list hive
    hive = [Bee(list(range(number_city))) for _ in range(number_bee)]
    number_employed_bee = math.ceil(number_bee * employed_bee_per)
    number_onlooker_bee = number_bee - number_employed_bee

    # Khởi tạo kết quả trả về của bài toán
    min_dis = math.inf
    path_res = []
    ### Gán role cho các loại bee
    # Employed bee
    for i in range(number_employed_bee):
        hive[i].role = 'E'
        random.shuffle(hive[i].path)
        hive[i].distance = total_distance(hive[i].path, distance_matrix)
        if min_dis > hive[i].distance:
            min_dis = hive[i].distance
            path_res = hive[i].path

    # Onlooker bee
    for i in range(number_onlooker_bee):
        hive[i + number_employed_bee].role = 'O'

    return hive, min_dis, list(path_res), number_employed_bee, number_onlooker_bee