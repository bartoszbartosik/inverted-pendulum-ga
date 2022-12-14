from enum import Enum

from population import Population


class StopCondition(Enum):
    ITERATIONS = 1
    FINAL_VALUE = 2


class GeneticAlgorithm:

    def __init__(self, objective_function, population_size, chromosome_size, gene_bounds, mutation_probability,
                 crossover_probability, crossover_rate):
        self.objfunction = objective_function

        self.population = Population(population_size=population_size,
                                     chromosome_size=chromosome_size,
                                     gene_bounds=gene_bounds,
                                     objective_function=objective_function,
                                     mutation_probability=mutation_probability,
                                     crossover_probability=crossover_probability,
                                     crossover_rate=crossover_rate)

        self.iteration = 1
        self.iterations = []

        self.individuals = []
        self.values = []

    def calculate(self, stop_condition, stop_value, save):

        if stop_condition == StopCondition.ITERATIONS:
            for i in range(stop_value):
                self.__compute()
        elif stop_condition == StopCondition.FINAL_VALUE:
            while abs(self.objfunction(self.population.fittest.chromosome)) > stop_value:
                self.__compute()

        print("SOLUTION: {}".format(self.individuals[self.values.index(max(self.values))].chromosome))
        print("VALUE: {}".format(max(self.values)))

        if save:
            file = open('results/ga.txt', 'a')
            file.write('\nParameters: [psize: {}; mprob: {}; crossprob: {}; crossrate: {}]'
                       .format(self.population.psize, self.population.mprobability, self.population.crossprobability,
                               self.population.crossrate))
            file.write('\niterations={}\nvalues={}\n'.format(self.iterations, self.values))
            file.close()

        return self.individuals[self.values.index(max(self.values))].chromosome

    def __compute(self):

        print("--- ITERATON {} ---".format(self.iteration))
        self.iterations.append(self.iteration)
        self.population.selection()
        self.population.crossover()
        self.population.mutation()

        # for individual in self.population.individuals:
        #     print(individual.chromosome)
        #     print(self.objfunction(individual.chromosome))

        self.population.findfittest()

        print('fittest: ' + str(self.population.fittest.chromosome))
        print('value: ' + str(self.population.fittest_val))

        self.iteration += 1
        self.individuals.append(self.population.fittest)
        self.values.append(self.population.fittest_val)

