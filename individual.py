import decimal
import sys
import random


class Individual:

    # CONSTRUCTOR
    def __init__(self, size=None, chromosome=None, gene_bounds=None):
        # Initialize chromosome
        if chromosome is None:
            self.chromosome = [0] * size
        else:
            self.chromosome = chromosome

        # Initialize gene boundaries
        if gene_bounds is None:
            self.gbounds = (-2 * sys.maxsize, 2 * sys.maxsize)
            self.dplaces = 0
        else:
            self.gbounds = gene_bounds
            d = decimal.Decimal(str(gene_bounds[1]))
            try:
                self.dplaces = len(str(d).split(".")[1])
            except:
                self.dplaces = 0

        self.size = len(self.chromosome)

    # INITIALIZE INDIVIDUAL
    def initialize(self):
        # Define lower and upperbound values for a single gene
        minvalue, maxvalue = self.gbounds
        # maxvalue = 2*sys.maxsize
        # bits = len(bin(maxvalue)[2:])
        for i in range(len(self.chromosome)):
            # self.gene[i] = bin(random.randint(minvalue, maxvalue))[2:].zfill(bits)
            # self.chromosome[i] = random.randint(minvalue*10.0, maxvalue*10.0)/10.0
            self.chromosome[i] = random.randint(minvalue*10**self.dplaces, maxvalue*10**self.dplaces)/(10**self.dplaces)

    def fitness(self, objectivefunction):
        # if value == 0:
        #     return 2*sys.maxsize
        # else:
        #     return 1/abs(value)
        return objectivefunction(self.chromosome)


    # IN CASE OF BINARY VARIABLE USAGE
    # def getvalue(self):
    #     values = []
    #
    #     for gene in self.gene:
    #         values.append(int(gene, 2))
    #
    #     return values

