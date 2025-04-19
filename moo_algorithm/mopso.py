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
import random
import warnings
from shapely.geometry import LineString
warnings.filterwarnings("ignore", category=RuntimeWarning)
warnings.filterwarnings("ignore", category=UserWarning)

class MOPSOPopulation(Population):
    def __init__(self, pop_size):
        super().__init__(pop_size)
        self.ParetoFront = []
        self.global_best = None

    def initialize(self):
        for ind in self.indivs:
            ind.velocity = [[0, 0] for _ in range(len(ind.chromosome))]
            ind.personal_best = ind.chromosome[:]
            ind.personal_best_objectives = ind.objectives
        self.update_global_best()

    def update_global_best(self):
        self.ParetoFront = self.fast_nondominated_sort(self.indivs)
        if len(self.ParetoFront) > 0:
            self.global_best = np.random.choice(self.ParetoFront[0]).chromosome[:]

    def fast_nondominated_sort(self, indi_list):
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
        return ParetoFront

    def update_personal_best(self):
        for individual in self.indivs:
            if check_dominate(individual.objectives, individual.personal_best_objectives):
                individual.personal_best = individual.chromosome[:]
                individual.personal_best_objectives = individual.objectives

    def update_velocity_and_position(self, w, c1, c2):
        for individual in self.indivs:
            r1 = np.random.rand(len(individual.chromosome))
            r2 = np.random.rand(len(individual.chromosome))

            list_nearest_global_best = []
            for p in individual.chromosome:
                min_dis = -1
                idx = -1
                for j, p1 in enumerate(self.global_best):
                    if min_dis == -1 or min_dis > (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2:
                        min_dis = (p1[0] - p[0]) ** 2 + (p1[1] - p[1]) ** 2
                        idx = j
                list_nearest_global_best.append(self.global_best[idx])
    
            for j in range(len(individual.chromosome)):
                individual.velocity[j][0] = w * individual.velocity[j][0] + c1 * r1[j] * (individual.personal_best[j][0] - individual.chromosome[j][0]) + c2 * r2[j] * (list_nearest_global_best[j][0] - individual.chromosome[j][0])
                individual.velocity[j][1] = w * individual.velocity[j][1] + c1 * r1[j] * (individual.personal_best[j][1] - individual.chromosome[j][1]) + c2 * r2[j] * (list_nearest_global_best[j][1] - individual.chromosome[j][1])
            new_path = []
            for j in range(len(individual.chromosome)):
                new_path.append((int(individual.chromosome[j][0] + individual.velocity[j][0]), int(individual.chromosome[j][1] + individual.velocity[j][1])))
            line = LineString(new_path)
            candidates = tree.query(line, predicate='intersects')
            if len(candidates) == 0:
                for j in range(len(individual.chromosome)):
                    individual.chromosome[j] = new_path[j]

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

def run_mopso(tree, obstacles, indi_list, pop_size, max_gen, w, min_w, c1, c2, cal_fitness):
    print("MOPSO")
    history = {}
    mopso_pop = MOPSOPopulation(pop_size)
    mopso_pop.pre_indi_gen(indi_list)
    for i in range(len(mopso_pop.indivs)):
        mopso_pop.indivs[i].objectives = cal_fitness(mopso_pop.indivs[i].chromosome, tree)

    mopso_pop.initialize()
    mopso_pop.update_personal_best()
    mopso_pop.update_global_best()

    print("Generation 0: Done")
    Pareto_store = [list(indi.objectives) for indi in mopso_pop.ParetoFront[0]]
    history[0] = Pareto_store

    for gen in range(max_gen):
        Pareto_store = []
        mopso_pop.update_velocity_and_position(w, c1, c2)
        w = max(w * 0.95, min_w)

        for i in range(len(mopso_pop.indivs)):
            mopso_pop.indivs[i].objectives = cal_fitness(mopso_pop.indivs[i].chromosome, tree)


        mopso_pop.update_personal_best()
        mopso_pop.update_global_best()
        mopso_pop.natural_selection()

        if (gen + 1) % 50 == 0:
            print(f"Generation {gen + 1}: Done")

        Pareto_store = [list(indi.objectives) for indi in mopso_pop.ParetoFront[0]]
        history[gen + 1] = Pareto_store

    print("MOPSO Done!")
    POP = []
    for ind in mopso_pop.indivs:
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
max_gen = 2000
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

run_mopso(tree, obstacles, indi_list, pop_size, max_gen, 0.9, 0.4, 1.5, 1.5, cal_objective)