import multiprocessing
import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/..")
from moo_algorithm.metric import cal_hv_front
from population import Population, Individual
import copy
import random
import numpy as np
from read_map import read_map_from_file
from space_segment import SegmentSpace
from RRT import RRT
from objective_solver import cal_objective, check_dominate
from geometry import path_crossover_operator, path_mutation_operator, fast_non_dominated_sort
from plot import plot_map
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)
warnings.filterwarnings("ignore", category=UserWarning)

class ReferencePoint(list):
    def __init__(self, *args):
        list.__init__(self, *args)
        self.associations_count = 0
        self.associations = []


def generate_reference_points(num_objs, num_divisions_per_obj=4):
    def gen_refs_recursive(work_point, num_objs, left, total, depth):
        if depth == num_objs - 1:
            work_point[depth] = left / total
            ref = ReferencePoint(copy.deepcopy(work_point))
            return [ref]
        else:
            res = []
            for i in range(left):
                work_point[depth] = i / total
                res = res + gen_refs_recursive(
                    work_point, num_objs, left - i, total, depth + 1
                )
            return res

    return gen_refs_recursive(
        [0] * num_objs,
        num_objs,
        num_objs * num_divisions_per_obj,
        num_objs * num_divisions_per_obj,
        0,
    )


def find_ideal_point(indivs):
    m = len(indivs[0].objectives)
    ideal_point = [np.infty]*m
    for indi in indivs:
        for i in range(m):
            ideal_point[i] = min(ideal_point[i], indi.objectives[i])
    return ideal_point


def find_extreme_points(individuals):
    return [
        sorted(individuals, key=lambda ind: ind.objectives[o])[-1]
        for o in range(len(individuals[0].objectives))
    ]


def construct_hyperplane(individuals, extreme_points):
    def has_duplicate_individuals(individuals):
        for i in range(len(individuals)):
            for j in range(i + 1, len(individuals)):
                if individuals[i].objectives == individuals[j].objectives:
                    return True
        return False

    num_objs = len(individuals[0].objectives)

    if has_duplicate_individuals(extreme_points):
        intercepts = [extreme_points[m].objectives[m] for m in range(num_objs)]
    else:
        b = np.ones(num_objs)
        A = [point.objectives for point in extreme_points]
        x = np.linalg.solve(A, b)
        intercepts = 1 / x
    return intercepts


def normalize_objective(individual, m, intercepts, ideal_point, epsilon=1e-20):
    if np.abs(intercepts[m] - ideal_point[m]) > epsilon:
        return individual.objectives[m] / (intercepts[m] - ideal_point[m])
    else:
        return individual.objectives[m] / epsilon


def normalize_objectives(individuals, intercepts, ideal_point):
    num_objs = len(individuals[0].objectives)

    for ind in individuals:
        ind.normalized_values = list(
            [
                normalize_objective(ind, m, intercepts, ideal_point)
                for m in range(num_objs)
            ]
        )
    return individuals


def perpendicular_distance(direction, point):
    k = np.dot(direction, point) / np.sum(np.power(direction, 2))
    d = np.sum(
        np.power(np.subtract(np.multiply(direction, [k] * len(direction)), point), 2)
    )
    return np.sqrt(d)


def associate(individuals, reference_points):
    num_objs = len(individuals[0].objectives)

    for ind in individuals:
        rp_dists = [
            (rp, perpendicular_distance(ind.objectives, rp))
            for rp in reference_points
        ]
        best_rp, best_dist = sorted(rp_dists, key=lambda rpd: rpd[1])[0]
        
        ind.reference_point = best_rp
        ind.ref_point_distance = best_dist
        
        best_rp.associations_count += 1
        best_rp.associations.append(ind)


def niching_select(individuals, k):
    if len(individuals) == k:
        return individuals

    ideal_point = find_ideal_point(individuals)
    extremes = find_extreme_points(individuals)
    
    intercepts = construct_hyperplane(individuals, extremes)
    normalize_objectives(individuals, intercepts, ideal_point)

    reference_points = generate_reference_points(len(individuals[0].objectives))
    associate(individuals, reference_points)

    res = []
    while len(res) < k:
        min_assoc_rp = min(reference_points, key=lambda rp: rp.associations_count)
        
        min_assoc_rps = [
            rp
            for rp in reference_points
            if rp.associations_count == min_assoc_rp.associations_count
        ]
        
        chosen_rp = min_assoc_rps[random.randint(0, len(min_assoc_rps) - 1)]

        if chosen_rp.associations:
            if chosen_rp.associations_count == 0:
                sel = min(
                    chosen_rp.associations, key=lambda ind: ind.ref_point_distance
                )
            else:
                sel = chosen_rp.associations[random.randint(0, len(chosen_rp.associations) - 1)]
            res += [sel]
            chosen_rp.associations.remove(sel) 
            chosen_rp.associations_count += 1 
            individuals.remove(sel) 
        else:
            reference_points.remove(chosen_rp)

    return res


def sel_nsga_iii(individuals, k):
    assert len(individuals) >= k

    if len(individuals) == k:
        return individuals

    fronts = fast_nondominated_sort(individuals)

    limit = 0
    res = []
    for f, front in enumerate(fronts):
        res += front
        if len(res) > k:
            limit = f
            break
    selection = []
    if limit > 0:
        for f in range(limit):
            selection += fronts[f]
    selection += niching_select(fronts[limit], k - len(selection))
    return selection


def fast_nondominated_sort(indi_list):
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


class NSGAIIIPopulation(Population):
    def __init__(self, pop_size):
        super().__init__(pop_size)
        self.ParetoFront = []
    def fast_nondominated_sort(self):
        self.ParetoFront = fast_nondominated_sort(self.indivs)

    def natural_selection(self):
        self.indivs = sel_nsga_iii(self.indivs, self.pop_size)


def run_nsga_iii(tree, obstacles, indi_list, pop_size, max_gen, crossover_operator, mutation_operator, 
                crossover_rate, mutation_rate, cal_fitness):
    print("NSGA-III")
    nsga_iii_pop = NSGAIIIPopulation(pop_size)
    nsga_iii_pop.pre_indi_gen(indi_list)

    history = {}

    for i in range(len(nsga_iii_pop.indivs)):
        nsga_iii_pop.indivs[i].objectives = cal_fitness(nsga_iii_pop.indivs[i].chromosome, tree)
    nsga_iii_pop.fast_nondominated_sort()

    print("Generation 0: Done")
    # Pareto_store = []
    # for indi in nsga_iii_pop.ParetoFront[0]:
    #     Pareto_store.append(list(indi.objectives))
    # history[0] = Pareto_store

    for gen in range(max_gen):
        # print("Bắt đầu tạo offspring")
        offspring = nsga_iii_pop.gen_offspring(tree, crossover_operator, mutation_operator, crossover_rate, mutation_rate, cal_fitness)
        # print("Bắt đầu tính fitness: ", len(offspring))
        for i in range(len(nsga_iii_pop.indivs)):
            nsga_iii_pop.indivs[i].objectives = cal_fitness(nsga_iii_pop.indivs[i].chromosome, tree)
    
        # print("Tinh fitness xong")
        
        nsga_iii_pop.indivs.extend(offspring)
        nsga_iii_pop.natural_selection()
        nsga_iii_pop.fast_nondominated_sort()
        print(f"Generation {gen+1}: Done")
        # print(len(nsga_iii_pop.ParetoFront[0]))
        if gen == max_gen - 1:
            Pareto_store = []
            for indi in nsga_iii_pop.ParetoFront[0]:
                Pareto_store.append(list(indi.objectives))
            history[gen+1] = Pareto_store
        

    print("NSGA-III Done!")
    POP = []
    for ind in nsga_iii_pop.indivs:
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

run_nsga_iii(tree, obstacles, indi_list, pop_size, max_gen, path_crossover_operator, path_mutation_operator, 0.5, 0.1, cal_objective)