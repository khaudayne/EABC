import multiprocessing
import sys
import os
import numpy as np
# Add the parent directory to the module search path
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/..")
from moo_algorithm.metric import cal_hv_front
from population import Population, Individual
from population import Population, Individual
from read_map import read_map_from_file
from space_segment import SegmentSpace
from RRT import RRT
from objective_solver import cal_objective, check_dominate
from geometry import fast_non_dominated_sort, path_crossover_operator, path_mutation_operator
from plot import plot_map
import random
import warnings
from shapely.geometry import LineString
warnings.filterwarnings("ignore", category=RuntimeWarning)
warnings.filterwarnings("ignore", category=UserWarning)

def init_weight_vectors_2d(pop_size):
    wvs = []
    for i in np.arange(0, 1 + sys.float_info.epsilon, 1 / (pop_size - 1)):
        wvs.append([i, 1 - i])
    return np.array(wvs)

def init_weight_vectors_3d(pop_size):
    wvs = []
    for i in np.arange(0, 1 + sys.float_info.epsilon, 1 / (pop_size - 1)):
        for j in np.arange(0, 1 + sys.float_info.epsilon, 1 / (pop_size - 1)):
            if i + j <= 1:
                wvs.append([i, j, 1 - i - j])
    return np.array(wvs)

def init_weight_vectors_4d(pop_size):
    wvs = []
    for i in np.arange(0, 1 + sys.float_info.epsilon, 1 / (pop_size - 1)):
        for j in np.arange(0, 1 + sys.float_info.epsilon, 1 / (pop_size - 1)):
            for k in np.arange(0, 1 + sys.float_info.epsilon, 1 / (pop_size - 1)):
                if i + j + k <= 1:
                    wvs.append([i, j, k, 1 - i - j - k])
    return np.array(wvs)


class MOEADPopulation(Population):
    def __init__(self, pop_size,  neighborhood_size, init_weight_vectors):
        super().__init__(pop_size)
        self.neighborhood_size = neighborhood_size
        self.external_pop = []
        self.weights = init_weight_vectors(self.pop_size)
        self.neighborhoods = self.init_neighborhood()

    def init_neighborhood(self):
        B = np.empty([self.pop_size, self.neighborhood_size], dtype=int)
        for i in range(self.pop_size):
            wv = self.weights[i]
            euclidean_distances = np.empty([self.pop_size], dtype=float)
            for j in range(self.pop_size):
                euclidean_distances[j] = np.linalg.norm(wv - self.weights[j])
            B[i] = np.argsort(euclidean_distances)[:self.neighborhood_size]
        return B

    def reproduction(self, tree, crossover_operator, mutation_operator, mutation_rate):
        offspring = []
        for i in range(self.pop_size):
            parent1, parent2 = np.random.choice(self.neighborhoods[i].tolist(), 2, replace=False)
            c1, c2 = crossover_operator(self.indivs[parent1].chromosome, self.indivs[parent2].chromosome, tree, True)
            off1 = Individual(c1)
            if np.random.rand() < mutation_rate:
                off1.chromosome = mutation_operator(off1.chromosome, tree)
            offspring.append(off1)
        return offspring
    
    def cal_value_ind(self, obj, w):
        value_indi = 0
        for i in range(len(w)):
            value_indi += w[i] * obj[i]
        return value_indi

    def natural_selection(self):
        self.indivs, O = self.indivs[:self.pop_size], self.indivs[self.pop_size:]
        for i in range(self.pop_size):
            indi = O[i]
            for j in self.neighborhoods[i]:
                if self.cal_value_ind(indi.objectives, self.weights[j]) < self.cal_value_ind(self.indivs[j].objectives, self.weights[j]):
                    self.indivs[j] = indi

    def update_external(self, indivs: list):
        for indi in indivs:
            old_size = len(self.external_pop)
            self.external_pop = [other for other in self.external_pop
                                 if not check_dominate(indi.objectives, other.objectives)]
            if old_size > len(self.external_pop):
                self.external_pop.append(indi)
                continue
            for other in self.external_pop:
                if check_dominate(other.objectives, indi.objectives):
                    break
            else:
                self.external_pop.append(indi)

def run_moead(tree, obstacles, indi_list, pop_size, max_gen, neighborhood_size, 
              init_weight_vectors, crossover_operator,mutation_operator, cal_fitness):
    print("MOEA/D")
    moead_pop = MOEADPopulation(pop_size, neighborhood_size, init_weight_vectors)
    moead_pop.pre_indi_gen(indi_list)
    history = {}
    for i in range(len(moead_pop.indivs)):
        moead_pop.indivs[i].objectives = cal_fitness(moead_pop.indivs[i].chromosome, tree)
    
    moead_pop.update_external(moead_pop.indivs)
    # moead_pop.update_weights(problem, moead_pop.indivs)
    print("Generation 0: Done")
    Pareto_store = []
    for indi in moead_pop.external_pop:
        Pareto_store.append(list(indi.objectives))
    history[0] = Pareto_store

    for gen in range(max_gen):
        offspring = moead_pop.reproduction(tree, crossover_operator, mutation_operator, 0.1)
        for i in range(len(offspring)):
            offspring[i].objectives = cal_fitness(offspring[i].chromosome, tree)

        moead_pop.update_external(offspring)
        moead_pop.indivs.extend(offspring)
        moead_pop.natural_selection()
        print("Generation {}: Done".format(gen + 1))
        Pareto_store = []
        for indi in moead_pop.external_pop:
            Pareto_store.append(list(indi.objectives))
        history[gen + 1] = Pareto_store

    print("MOEA/D Done!")
    POP = []
    for ind in moead_pop.indivs:
        POP.append(ind.chromosome)
    NDS_archive_idx, POP_ns_idx, list_obj = fast_non_dominated_sort(POP, tree)
    print("\nEND algorithm, show result below:\n")
    for i in range(len(NDS_archive_idx)):
        path = POP[NDS_archive_idx[i]]
        print("\nRount {}: {}".format(i + 1, path))
        obj = list_obj[NDS_archive_idx[i]]
        print("Have objective value is: {}".format(obj))
    plot_map(POP[NDS_archive_idx[random.randint(0, len(NDS_archive_idx) - 1)]], obstacles)
    
    return history

start = (50, 50)
goal = (378, 456)
pop_size = 50
max_gen = 100
path_data = "data/map3.txt"
indi_list = []
map_size, obstacles, tree = read_map_from_file(path_data)
rrt = RRT(start, goal, map_size, tree, step_size=15, max_iter=10000)
space_segment = SegmentSpace(start, goal, 15, map_size, tree, number_try=25)
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

run_moead(tree, obstacles, indi_list, pop_size, max_gen, 5, init_weight_vectors_2d, path_crossover_operator, path_mutation_operator, cal_objective)