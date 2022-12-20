import argparse

from inverted_pendulum import InvertedPendulum
from geneticalgorithm import GeneticAlgorithm
from geneticalgorithm import StopCondition
from lqr4ip import LQR4IP

import numpy as np


def main():

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

    ###########################
    # INVERTED PENDULUM SETUP #
    ###########################

    ######################
    # PHYSICAL CONSTANTS #
    # Pendulum
    L = 0.8     # [m]
    m = 0.3     # [kg]
    b = 0.05     # [kg/s]

    # Cart
    M = 0.6     # [kg]
    B = 0.1     # [1/s]

    # Gravitational acceleration
    g = -9.81   # [m/s^2]

    ######################
    # INITIAL CONDITIONS #
    th = 30.0      # [deg]
    dth = 100.0     # [deg/s]
    x = 3.0       # [m]
    dx = -2.0     # [m/s]
    th = 15.0      # [deg]
    dth = 0.0     # [deg/s]
    x = 0.0       # [m]
    dx = 0.0     # [m/s]

    # State vector
    state = np.array([th, dth, x, dx])

    ##############
    # SIMULATION #
    # Time
    dt = 0.075
    t_max = 15
    t = [0.0, t_max, dt]

    # Desired position
    x_pos = [
        # [5, 10, t_max],     # time [s]
        # [0, 1, -1]      # reference position [m]
        [0, t_max],     # time [s]
        [0, 0]      # reference position [m]
    ]

    ##################
    # INITIALIZATION #
    inverted_pendulum = InvertedPendulum(
        pendulum_length=L,
        pendulum_mass=m,
        pendulum_friction=b,
        cart_mass=M,
        cart_friction=B,
        initial_state=state,
        desired_position=x_pos,
        g=g,
        time_domain=t
    )

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

    #####################
    # SYSTEM SIMULATION #
    #####################

    #################
    # PLOT SETTINGS #
    animate_plot = True
    save_plot = True
    relative_path = ""
    # relative_path = ""
    filename = ""

    filenameprefix = "/state[{};{};{};{}]-".format(th, dth, x, dx)

    ##############
    # CONTROLLER #
    # Choose either 'LQR' or 'GA'
    controller = 'LQR'

    ##############################
    # LINEAR QUADRATIC REGULATOR #
    ##############################

    ###########
    # WEIGHTS #
    Q = np.diag([2000, 0, 110, 0])
    R = 1

    ##################
    # INITIALIZATION #
    lqr = LQR4IP(inverted_pendulum, Q, R)

    ########################
    # REFERENCE CONTROLLER #
    K_ref = lqr.K

    if controller.__eq__('LQR'):
        #######
        # SET #
        K_sol = K_ref

        ####################
        # PLOT NAME PREFIX #
        filenameprefix += "lqr[{};{}]-k[{};{};{};{}]"\
                              .format(str(Q[0][0]), str(Q[2][2]),
                                      round(K_sol[0], 2), round(K_sol[1], 2), round(K_sol[2], 2), round(K_sol[3], 2))

    elif controller.__eq__('GA'):
        ###############################
        # GENETIC ALGORITHM REGULATOR #
        ###############################

        ######################
        # OBJECTIVE FUNCTION #
        def objfunction(K):

            inverted_pendulum.K = K
            inverted_pendulum.calculate()

            correction_coeff = inverted_pendulum.t[-1]**2
            # theta_integral = inverted_pendulum.get_theta_integral()
            x_integral = inverted_pendulum.get_x_integral()
            theta_integral = 1
            # x_integral = 1

            return correction_coeff*(1/(theta_integral*x_integral))

        ##################
        # INITIALIZATION #
        ga = GeneticAlgorithm(objective_function=objfunction,
                              population_size=20,
                              chromosome_size=4,
                              gene_bounds=(0, 100.01),
                              mutation_probability=0.2,
                              crossover_probability=0.4,
                              crossover_rate=0.8)

        K_sol = ga.calculate(StopCondition.ITERATIONS, 100)

        ####################
        # PLOT NAME PREFIX #
        filenameprefix += "ga[psize{};mprob{};crossprob{};crossrate{}]-k[{};{};{};{}]"\
            .format(ga.population.psize, ga.population.mprobability,
                    ga.population.crossprobability, ga.population.crossrate,
                    round(K_sol[0], 2), round(K_sol[1], 2), round(K_sol[2], 2), round(K_sol[3], 2))

        ga.plot(save=save_plot, filename=relative_path+filenameprefix+filename+"-generations")

    else:
        ################
        # NO REGULATOR #
        ################
        K_sol = [0, 0, 0, 0]

        ####################
        # PLOT NAME PREFIX #
        filenameprefix += "nocontrol"

    ##############
    # SIMULATION #

    # K_sol = [43.4, 25.6, 0.6, 7.0]
    # K_sol = [50, 20, 3.1, 4.8]
    # K_sol = [41, 19, 1.5, 3.8]
    # K_sol = [38, 4, 10, 2]
    # K_sol = [44.2, 16.5, 0.0, 0.1]      # smooth theta control
    # K_sol = [88.1, 19.5, 9.3, 13.5]     # nice result
    # K_sol = [63.11, 18.3, 6.68, 9.35]   # nice result
    # K_sol = [83.72, 25.1, 10.98, 13.32]   # nice result
    # K_sol = [99.06222414, 23.29486085, 10.0, 13.66361755]   # LQR

    # K_sol = [57.45, 91.77, -0.38, 65.55]    # 1
    # K_sol = [76.88, 91.95, 2.92, 42.29]     # 5
    # K_sol = [76.88, 91.95, 2.92, 28.24]     # 10
    # K_sol = [98.07, 91.95, 17.17, 42.29]    # 20
    # K_sol = [98.07, 73.16, 14.32, 38.19]    # 50
    # K_sol = [98.07, 42.0, 13.78, 24.16]     # 75

    ######################################
    # SOLVE FOR REFERENCE LQR CONTROLLER #
    # inverted_pendulum.K = K_ref
    # ref_solution = inverted_pendulum.calculate()

    inverted_pendulum.K = K_sol
    inverted_pendulum.calculate()
    inverted_pendulum.plot(animate=animate_plot, save=save_plot, filename=relative_path+filenameprefix+filename,
                           reference_solution=None)


if __name__ == '__main__':
    main()
