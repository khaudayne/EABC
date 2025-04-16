import numpy as np
from copy import deepcopy

class Individual:
    def __init__(self, chromosome = None):
        self.chromosome = chromosome
        self.objectives = None # Objectives vector

        self.domination_count = None # be dominated 
        self.dominated_solutions = None # dominate
        self.crowding_distance = None
        self.rank = None
    
    def gen_random(self, problem, create_solution):
        self.chromosome = create_solution(problem)

    # Dominate operator
    def dominates(self, other_individual):
        tolerance= 0
        and_condition = True
        or_condition = False
        for first, second in zip(self.objectives, other_individual.objectives):
            and_condition = and_condition and (first <= second + tolerance)
            or_condition = or_condition or (first < second - tolerance)
        return and_condition and or_condition
    
    def repair(self):
        for i in range(len(self.chromosome)):
            if self.chromosome[i] > np.pi:
                self.chromosome[i] = np.pi
            elif self.chromosome[i] < -np.pi:
                self.chromosome[i] = -np.pi
    
    

class Population:
    def __init__(self, pop_size):
        self.pop_size = pop_size
        self.indivs = []

    def pre_indi_gen(self, indi_list):
        if len(indi_list) != self.pop_size:
            raise ValueError("The length of the list must be equal to the population size")
        self.indivs = deepcopy(indi_list)

    def gen_offspring(self, tree, crossover_operator, mutation_operator, crossover_rate, mutation_rate, cal_fitness):
        offspring = []
        for i in range(self.pop_size):
            parent1, parent2 = np.random.choice(self.indivs, 2, replace = False)
            if np.random.rand() < crossover_rate:
                c1, c2 = crossover_operator(parent1.chromosome, parent2.chromosome, tree, True)
                off1 = Individual(c1)
                off2 = Individual(c2)
            else:
                off1 = Individual(parent1.chromosome[:])
                off2 = Individual(parent2.chromosome[:])
            if np.random.rand() < mutation_rate:
                off1.chromosome = mutation_operator(off1.chromosome, tree)
                off2.chromosome = mutation_operator(off2.chromosome, tree)
            off1.objectives = cal_fitness(off1.chromosome, tree)
            off2.objectives = cal_fitness(off2.chromosome, tree)
            offspring.append(off1)
            offspring.append(off2)
        return offspring
    