import decimal
import sys
import random

from geneticalgorithm.individual import Individual


class Population:

    # CONSTRUCTOR
    def __init__(self, objective_function, population_size, chromosome_size, gene_bounds, mutation_probability,
                 crossover_probability, crossover_rate):
        # Initialize size of population
        self.psize = population_size

        # Initialize each individual's size
        self.isize = chromosome_size

        # Initialize objective function
        self.objectivefunction = objective_function

        # Initialize gene boundaries
        self.gbounds = gene_bounds
        d = decimal.Decimal(str(gene_bounds[1]))
        try:
            self.dplaces = len(str(d).split(".")[1])
        except:
            self.dplaces = 0

        # Initialize mutation probability
        self.mprobability = mutation_probability

        # Initialize crossover rate
        self.crossprobability = crossover_probability

        # Initialize crossover rate
        self.crossrate = crossover_rate

        # Initialize individuals array
        self.individuals = [0] * self.psize

        for i in range(len(self.individuals)):
            individual = Individual(gene_bounds=self.gbounds, size=self.isize)
            individual.initialize()
            self.individuals[i] = individual

        # Initialize fittest individual
        self.fittest = None
        self.fittest_val = 0

        self.findfittest()

    # SELECTION MECHANISM
    def selection(self):
        # Initialize fitnesses values array
        individualsfitnesses = []

        # Get fitness of each individual and append it to fitnesses array
        for individual in self.individuals:
            fitness = individual.fitness(self.objectivefunction)
            individualsfitnesses.append(fitness)

        # Calculate sum of each individual's fitnesses
        fitsum = sum(individualsfitnesses)

        # Initialize relative fitnesses and selection probability arrays for each individual
        relativefitnesses = []
        selectionprobabilities = []
        probability = 0

        # Iterate over each individual fitnesses
        for individualfitness in individualsfitnesses:
            # Calculate relative fitness of the individual
            relativefitness = individualfitness/fitsum
            relativefitnesses.append(relativefitness)

            # Calculate the selection probability of the individual
            probability = probability + relativefitness
            selectionprobabilities.append(probability)

        # Extend probabilities array to include 0 probability
        selectionprobabilities.insert(0, 0)

        # Initialize selected individuals array
        selected = []

        # Iterate over population size
        for i in range(self.psize):
            # Choose random number between 0 and 1
            selection = random.uniform(0, 1)
            # Iterate over probabilities intervals
            for j in range(self.psize):
                # If the randomly selected number fits in the current probability intervals, add proper
                # individual as a selected for a new generation
                if selectionprobabilities[j] < selection < selectionprobabilities[j + 1]:
                    selectedindividual = Individual(chromosome=self.individuals[j].chromosome)
                    selected.append(selectedindividual)
                    break

        # Set selected individuals as a new generation
        self.individuals = selected.copy()

    # CROSSOVER MECHANISM
    def crossover(self):

        if random.uniform(0, 1) < self.crossprobability:

            indnum2cross = round(round(self.crossrate*self.psize)/2)*2
            ind2cross = random.sample(self.individuals, indnum2cross)

            for i in range(0, len(ind2cross)-1, 2):

                # Determine crossover point
                crosspoint = random.randint(1, self.isize - 1)

                # Perform crossover
                for j in range(crosspoint, self.isize):
                    temp = ind2cross[i].chromosome[j]
                    ind2cross[i].chromosome[j] = ind2cross[i+1].chromosome[j]
                    ind2cross[i+1].chromosome[j] = temp

    # MUTATION MECHANISM
    def mutation(self):
        # Iterate over individuals
        for individual in self.individuals:
            # Check probability of mutation
            if random.uniform(0, 1) < self.mprobability:
                # Extract individual's gene
                gene = individual.chromosome.copy()

                # Determine randomly mutation point
                mutationpoint = random.randint(0, self.isize - 1)

                # Perform mutation on the selected point
                # gene[mutationpoint] = random.randint(self.gbounds[0]*10.0, self.gbounds[1]*10.0)/10.0
                gene[mutationpoint] = random.randint(
                    self.gbounds[0]*10**self.dplaces, self.gbounds[1]*10**self.dplaces)/(10**self.dplaces)

                # Set mutated gene to the individual
                individual.chromosome = gene

    # GET THE FITTEST INDIVIDUAL
    def findfittest(self):

        # Assume minimum fitness
        fittest = -2*sys.maxsize

        # Assume there's no fittest individual
        fittestindividual = None

        # Iterate over individuals
        for individual in self.individuals:
            # If individual's fitness is greater than previous, set it as the fittest one
            indfitness = individual.fitness(self.objectivefunction)
            if indfitness > fittest:
                fittestindividual = individual
                fittest = indfitness

        self.fittest = fittestindividual
        self.fittest_val = fittest




