import os
import random
from typing import List
import yaml

from environment import Environment, Point

class GeneticSolver:
    def __init__(self, num_genes, population_size, max_generations, environment, individual_mutation = True):
        self.num_genes = num_genes
        self.population_size = population_size
        self.max_generations = max_generations
        self.environment: Environment = environment
        self.default_distance = self.environment.maklink.path_length
        self.default_path_distances = [0.0] * (num_genes)
        for i in range(num_genes):
            self.default_path_distances[i] = self.environment.maklink.maklink_distances[self.environment.maklink.path[i+1]]
        self.mutation_rate = 0.05
        self.individual_mutation = individual_mutation
        self.crossover_rate = 0.8
        self.population = self.initialize_population()

    def initialize_population(self):
        return [[random.uniform(0.0, 1.0) for _ in range(self.num_genes)] for _ in range(self.population_size)]

    def crisscross(self, parent1, parent2):
        # Classic crossover
        crossover_point = random.randint(1, self.num_genes - 1)
        child1 = parent1[:crossover_point] + parent2[crossover_point:]
        child2 = parent2[:crossover_point] + parent1[crossover_point:]
        return child1, child2

    def mutate(self, individual):
        # collect mutation probability based on path influence of point
        if self.individual_mutation:
            mutation_probabilities = self.evaluate_path_sections(individual)
        else:
            mutation_probabilities = [self.mutation_rate]*(self.num_genes)
        # Mutate each gene with a new value from a uniform distribution
        mutated_individual = [random.uniform(0.0, 1.0) if random.random() < mutation_probabilities[i] else individual[i] for i in range(self.num_genes)]
        return mutated_individual

    def roulette_wheel_selection(self, fitness_scores: List[float]):
        # Select parents based on a roulette-wheel proportional to fitness
        total_fitness = sum(fitness_scores)
        population = []
        for _ in range(self.population_size):
            spin = random.uniform(0, total_fitness)
            current_sum = 0
            for i, score in enumerate(fitness_scores):
                current_sum += score
                if current_sum >= spin:
                    population.append(self.population[i])
                    break
        self.population = population
    
    def evaluate(self, individual):
        return max((self.default_distance*0.01)**5, (self.default_distance - self.environment.calculate_path_length(individual))**5)

    def evaluate_path_sections(self, individual, printsmth=False):
        mutation_probabilities = []
        for i in range(self.num_genes):
            prev_section = self.environment.calculate_path_length(individual,i,i+1)
            prev_section_default = self.default_path_distances[0] if i==0 else self.default_path_distances[i]-self.default_path_distances[i-1]
            next_section = self.environment.calculate_path_length(individual,i+1,i+2)
            next_section_default = self.default_distance - self.default_path_distances[i] if i==self.num_genes-1 else self.default_path_distances[i+1]-self.default_path_distances[i]
            
            mutation_probability = (prev_section / prev_section_default + next_section / next_section_default) / 2 * self.mutation_rate
                        
            if printsmth and i==3:
                print("Prev section: {:.2f} vs. {:.2f}\nNext section: {:.2f} vs. {:.2f}\nRate: {:.2f}".format(prev_section, prev_section_default, next_section, next_section_default, mutation_probability))

            # if i > 0: previous_path_distance = self.default_path_distances[i-1]
            # else: previous_path_distance = 0.0
            # if i<self.num_genes-1: posterior_path_distance = self.default_path_distances[i+1]
            # else: posterior_path_distance = self.default_distance
            # #mutation_probability = (self.environment.calculate_path_length(individual,0,i+1) / self.default_path_distances[i] + self.environment.calculate_path_length(individual,i+1,-1) / (self.default_distance - self.default_path_distances[i])) / 2 * self.mutation_rate
            # mutation_probability = ((self.environment.calculate_path_length(individual,0,i+1)-self.environment.calculate_path_length(individual,0,i)) / (self.default_path_distances[i]-previous_path_distance) + (self.environment.calculate_path_length(individual,i+2,-1)-self.environment.calculate_path_length(individual,i+1,-1)) / (posterior_path_distance - self.default_path_distances[i])) / 2 * self.mutation_rate

            mutation_probabilities.append(mutation_probability)

        return mutation_probabilities

    def all_the_same(self):
        individual = self.population[0]
        for i in range(1,self.population_size):
            for j in range(self.num_genes):
                if self.population[i][j] != individual[j]:
                    return False
        return True

    def crossover(self):
        indices = [i for i in range(self.population_size)]
        random.shuffle(indices)
        single = True
        for i in indices:
            if random.random() < self.crossover_rate:
                single = not single
                if not single:
                    parent1 = i
                else:
                    parent2 = i
                    child1, child2 = self.crisscross(self.population[parent1], self.population[parent2])
                    self.population[parent1] = child1
                    self.population[parent2] = child2
                    
    def evolve(self):
        for generation in range(self.max_generations):
            # elitism
            scores = [self.evaluate(individual) for individual in self.population]
            elitist_score = max(scores)
            elitist = self.population[scores.index(elitist_score)]
            # roulette-wheel selection
            self.roulette_wheel_selection(scores)
            # Homogenity testing
            if self.all_the_same():
                print("Generación {} homogena.".format(generation))
                break
            # Crossover
            self.crossover()
            # Mutation
            self.population = [self.mutate(child) for child in self.population]
            # Evaluate fitness
            scores = [self.evaluate(individual) for individual in self.population]
            # elitism
            self.population[scores.index(min(scores))] = elitist
            scores[scores.index(min(scores))] = elitist_score
            if generation % 10 == 0:
                print("Generation: ", generation, " | Max. Fitness: {:.2f}".format(max(scores)))
                #self.environment.visualize_environment(self.population[scores.index(max(scores))])
        print("=======================================================\nDistancia original: {:.2f} | Distancia optimizada: {:.2f}\n=======================================================".format(self.default_distance,min(length for length in (self.environment.calculate_path_length(individual) for individual in self.population))))
        return [self.evaluate(individual) for individual in self.population]
    
    
def create_environment(x, y, obstacles, start_point, end_point):
    env = Environment(x, y)
    for obstacle in obstacles:
        env.add_obstacle(obstacle)
    # Calculating Maklink graph
    env.calculate_maklink()
    env.add_start_point(start_point)
    env.add_end_point(end_point)
    env.maklink.dijkstra()
    return env
    
def run_genetic_algorithm(env: Environment, population_size, max_generations, visualize = True, individual_solution = True):
    solver = GeneticSolver(len(env.maklink.path)-2, population_size, max_generations, env, individual_solution)
    scores = solver.evolve()

    # rounded_scores = ['%.2f' % elem for elem in scores]
    # print("\nSolution: {}".format(rounded_scores))
    # solver.evaluate_path_sections(solver.population[scores.index(max(scores))],printsmth=True)

    solution = solver.population[scores.index(max(scores))]

    if visualize:
        env.visualize_environment(solution, True)
    return env.calculate_path_length(solution)

def load_and_create_environments(yaml_file: str):
    with open(os.path.abspath(yaml_file), 'r') as file:
        environments = yaml.safe_load(file)

    for environment_data in environments:
        env_params = environment_data['environment']
        x = env_params['x']
        y = env_params['y']
        start_point = Point(env_params['start_point']['x'], env_params['start_point']['y'])
        end_point = Point(env_params['end_point']['x'], env_params['end_point']['y'])
        obstacles = [
            [Point(vertex['x'], vertex['y']) for vertex in obstacle]
            for obstacle in env_params['obstacles']
        ]

        env = create_environment(x, y, obstacles, start_point, end_point)
        run_genetic_algorithm(env, 200, 100)

def load_and_run_average_scores(yaml_file: str):
    with open(os.path.abspath(yaml_file), 'r') as file:
        environments = yaml.safe_load(file)

    for environment_data in environments:
        env_params = environment_data['environment']
        x = env_params['x']
        y = env_params['y']
        start_point = Point(env_params['start_point']['x'], env_params['start_point']['y'])
        end_point = Point(env_params['end_point']['x'], env_params['end_point']['y'])
        obstacles = [
            [Point(vertex['x'], vertex['y']) for vertex in obstacle]
            for obstacle in env_params['obstacles']
        ]

        env = create_environment(x, y, obstacles, start_point, end_point)
        base_length = env.maklink.path_length
        avg_solution_length = 0
        avg_individual_solution_length = 0
        for i in range(20):
            avg_solution_length += run_genetic_algorithm(env, 200, 100, visualize=False, individual_solution=False)
            avg_individual_solution_length += run_genetic_algorithm(env, 200, 100, visualize=False, individual_solution=True)
        print("Avg. Solver Score (normal mutation): {:.2f}".format(avg_solution_length))
        print("Avg. Solver Score (individual mutation): {:.2f}".format(avg_individual_solution_length))
        print()

if __name__ == "__main__":
    #load_and_create_environments("./environments.yaml")
    load_and_create_environments("./environment4.yaml")
    #load_and_run_average_scores("./environments.yaml")