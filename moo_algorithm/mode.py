import sys
import os
import numpy as np
# Add the parent directory to the module search path
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/..")
from moo_algorithm.metric import cal_hv_front
from population import Population, Individual
from read_map import read_map_from_file
from space_segment import SegmentSpace
from RRT import RRT
from objective_solver import cal_objective, check_dominate
from geometry import fast_non_dominated_sort
from plot import plot_map
from shapely.geometry import LineString
import random
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)
warnings.filterwarnings("ignore", category=UserWarning)

class MODEPopulation(Population):
    def __init__(self, pop_size):
        super().__init__(pop_size)
        self.ParetoFront = []
    

    def fast_nondominated_sort_crowding_distance(self, indi_list):
        ParetoFront = [[]]
        for individual in indi_list:
            individual.domination_count = 0
            individual.dominated_solutions = []
            for other_individual in indi_list:
                if individual.dominates(other_individual):
                    individual.dominated_solutions.append(other_individual)
                elif other_individual.dominates(individual):
                    individual.domination_count += 1
            if individual.domination_count == 0:
                individual.rank = 0
                ParetoFront[0].append(individual)
        i = 0
        while len(ParetoFront[i]) > 0:
            temp = []
            for individual in ParetoFront[i]:
                for other_individual in individual.dominated_solutions:
                    other_individual.domination_count -= 1
                    if other_individual.domination_count == 0:
                        other_individual.rank = i + 1
                        temp.append(other_individual)
            i = i + 1
            ParetoFront.append(temp)
        for front in ParetoFront:
            self.calculate_crowding_distance(front)
        return ParetoFront

    def calculate_crowding_distance(self, front):
        if len(front) > 0:
            solutions_num = len(front)
            for individual in front:
                individual.crowding_distance = 0

            for m in range(len(front[0].objectives)):
                front.sort(key=lambda individual: individual.objectives[m])
                front[0].crowding_distance = 10 ** 9
                front[solutions_num - 1].crowding_distance = 10 ** 9
                m_values = [individual.objectives[m] for individual in front]
                scale = max(m_values) - min(m_values)
                if scale == 0: scale = 1
                for i in range(1, solutions_num - 1):
                    front[i].crowding_distance += (front[i + 1].objectives[m] - front[i - 1].objectives[m]) / scale

    # Crowding Operator
    def crowding_operator(self, individual, other_individual):
        if (individual.rank < other_individual.rank) or \
                ((individual.rank == other_individual.rank) and (
                        individual.crowding_distance > other_individual.crowding_distance)):
            return 1
        else:
            return -1

    def natural_selection(self):
        self.ParetoFront = self.fast_nondominated_sort_crowding_distance(self.indivs)
        new_indivs = []
        new_fronts = []
        front_num = 0
        while len(new_indivs) + len(self.ParetoFront[front_num]) <= self.pop_size:
            new_indivs.extend(self.ParetoFront[front_num])
            new_fronts.append(self.ParetoFront[front_num])
            if len(new_indivs) == self.pop_size:
                break
            front_num += 1
        self.calculate_crowding_distance(self.ParetoFront[front_num])
        self.ParetoFront[front_num].sort(key=lambda individual: individual.crowding_distance, reverse=True)
        number_remain = self.pop_size - len(new_indivs)
        new_indivs.extend(self.ParetoFront[front_num][0:number_remain])
        new_fronts.append(self.ParetoFront[front_num][0:number_remain])
        self.ParetoFront = new_fronts
        self.indivs = new_indivs

    def gen_offspring_de(self, F, CR, tree):
        offspring = []
        for i in range(self.pop_size):
            indi1, indi2 = np.random.choice(self.indivs, 2, replace = False)
            off = rand_1(self.indivs[i], indi1, indi2, F, CR, tree)
            if off != None:
                offspring.append(off)
        return offspring
        


import random
def rand_1(indi : Individual, indi1: Individual, indi2: Individual, F, CR, tree):
    new_indi = Individual([])
    l1 = []
    l2 = []
    for p in indi.chromosome:
        min_dis = -1
        idx = -1
        for i in range(len(indi1.chromosome)):
            p1 = indi1.chromosome[i]
            if min_dis == -1 or min_dis > (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2:
                min_dis = (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2
                idx = i
        l1.append(indi1.chromosome[idx])

        min_dis = -1
        idx = -1
        for i in range(len(indi2.chromosome)):
            p1 = indi2.chromosome[i]
            if min_dis == -1 or min_dis > (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2:
                min_dis = (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2
                idx = i
        l2.append(indi2.chromosome[idx])
    
    for i, p in enumerate(indi.chromosome):
        if random.random() > CR:
            new_indi.chromosome.append(p)
        else:
            new_indi.chromosome.append((int(p[0] + F * (l1[i][0] - l2[i][0])), int(p[1] + F * (l1[i][1] - l2[i][1]))))
    line = LineString([p for p in new_indi.chromosome])
    candidates = tree.query(line, predicate='intersects')
    if len(candidates) > 0:
       return None
    return new_indi


def run_mode(tree, obstacles, indi_list, pop_size, max_gen, F, CR, cal_fitness):
    print("MODE")
    history = {}
    mode_pop = MODEPopulation(pop_size)
    mode_pop.pre_indi_gen(indi_list)

    for i in range(len(mode_pop.indivs)):
        mode_pop.indivs[i].objectives = cal_fitness(mode_pop.indivs[i].chromosome, tree)
    mode_pop.natural_selection()
    print("Generation 0: Done")
    Pareto_store = []
    for indi in mode_pop.ParetoFront[0]:
        Pareto_store.append(list(indi.objectives))
    history[0] = Pareto_store


    for gen in range(max_gen):
        Pareto_store = []
        offspring = mode_pop.gen_offspring_de(F, CR, tree)
        for i in range(len(offspring)):
            offspring[i].objectives = cal_fitness(offspring[i].chromosome, tree)
    
        mode_pop.indivs.extend(offspring)
        mode_pop.natural_selection()
        print("Generation {}: Done".format(gen + 1))
        for indi in mode_pop.ParetoFront[0]:
            Pareto_store.append(list(indi.objectives))
        history[gen + 1] = Pareto_store

    print("MODE Done!")
    POP = []
    for ind in mode_pop.indivs:
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

run_mode(tree, obstacles, indi_list, pop_size, max_gen, 0.5, 0.9, cal_objective)