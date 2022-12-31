from inverted_pendulum import InvertedPendulum
from geneticalgorithm import GeneticAlgorithm
from geneticalgorithm import StopCondition
from lqr4ip import LQR4IP
from plots import Plots

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

    # PHYSICAL VARIABLES #
    b = [
        [-180, -5, 1, 180],     # angle [deg]
        [0.05, 0.05, 0.075, 0.05]          # friction [-]
    ]

    # B = [
    #     [-2, -1.2, -0.8, 0, 1, 1.5, 5],     # position [m]
    #     [0.1, 0.1, 0.05, 0.08, 0.1, 0.15, 0.1]          # friction [-]
    # ]

    # B = [
    #     [-2, -1.1, -0.9, -0.2, 0, 1, 1.2, 5],     # position [m]
    #     [0.1, 0.1, 0.15, 0.1, 0.2, 0.1, 0.02, 0.1]          # friction [-]
    # ]

    # m = [
    #     [1, 3, 5, 8, 12, 15], # time [s]
    #     [0.3, 0.25, 0.2, 0.3, 0.35, 0.3]  # mass [kg]
    # ]

    # m = [
    #     [2, 4, 8, 12, 15], # time [s]
    #     [0.2, 0.3, 0.1, 0.5, 0.3]  # mass [kg]
    # ]

    ######################
    # INITIAL CONDITIONS #
    th = 30.0      # [deg]
    dth = 100.0     # [deg/s]
    x = 3.0       # [m]
    dx = -2.0     # [m/s]

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
        [5, 10, t_max],     # time [s]
        [0, 1, -1]      # reference position [m]
        # [0, t_max],     # time [s]
        # [0, 0]      # reference position [m]
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
    relative_path = "alternative scenarios/var pendulums friction"
    filename = "-animation"

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

    elif controller.__eq__('GAs'):
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
            x_integral, _ = inverted_pendulum.get_x_integral()
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
                              crossover_rate=0.2)

        K_sol = ga.calculate(StopCondition.ITERATIONS, 100, save=True)

        ####################
        # PLOT NAME PREFIX #
        filenameprefix += "ga[psize{};mprob{};crossprob{};crossrate{}]-k[{};{};{};{}]" \
            .format(ga.population.psize, ga.population.mprobability,
                    ga.population.crossprobability, ga.population.crossrate,
                    round(K_sol[0], 2), round(K_sol[1], 2), round(K_sol[2], 2), round(K_sol[3], 2))

        plots = Plots()
        plots.plot_ga_generations(geneticalgorithm=ga,
                                  filename=relative_path + filenameprefix + filename)
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

    # VAR POPULATION SIZE #
    K_PS_5 = [74.75, 24.76, 4.07, 14.5]
    K_PS_10 = [85.15, 70.88, 15.06, 35.68]
    K_PS_20 = [89.94, 20.16, 7.92, 13.65]
    K_PS_50 = [55.69, 13.62, 5.36, 7.6]

    # VAR OBJECTIVE FUNCTION #
    K_TH = [96.71, 24.68, -0.27, 17.62]
    K_X = [-6.94, 10.53, -94.52, -64.39]
    K_X_TH_DXLIM = [91.86, 46.94, 7.79, 28.22]
    K_X_TH_THLIM = [99.51, 74.77, 11.75, 46.67]

    # VAR MUTATION PROBABILITY #
    K_M01 = [91.13, 59.67, 13.03, 21.13]
    K_M02 = [89.94, 20.16, 7.92, 13.65]
    K_M04 = [99.66, 82.03, 10.92, 18.36]
    K_M08 = [65.91, 20.02, 8.3, 11.21]

    # VAR CROSSOVER PROBABILITY #
    K_CP01 = [94.32, 35.78, 14.8, 20.23]
    K_CP02 = [98.52, 42.34, 17.02, 24.26]
    K_CP04 = [89.94, 20.16, 7.92, 13.65]
    K_CP08 = [77.94, 27.5, 8.95, 12.75]

    # VAR CROSSOVER RATE #
    K_CR01 = [92.15, 48.63, 18.6, 26.38]
    K_CR02 = [89.94, 20.16, 7.92, 13.65]
    K_CR04 = [84.31, 24.19, 10.35, 13.73]
    K_CR08 = [93.41, 33.83, 4.71, 19.94]

    #######################################
    # CREATE REFERENCE INVERTED PENDULUMS #
    ref_ip_lqr = inverted_pendulum.copy()
    ref_ip_lqr.K = K_ref
    # ref_ip1 = inverted_pendulum.copy()
    # ref_ip1.K = K_PS_5
    # ref_ip2 = inverted_pendulum.copy()
    # ref_ip2.K = K_PS_10
    # ref_ip3 = inverted_pendulum.copy()
    # ref_ip3.K = K_PS_20
    # ref_ip4 = inverted_pendulum.copy()
    # ref_ip4.K = K_PS_50

    inverted_pendulum.K = K_sol
    inverted_pendulum.calculate()

    plots = Plots()

    # plots.plot_inverted_pendulum(inverted_pendulum=inverted_pendulum,
    #                              animate=animate_plot,
    #                              filename=relative_path+filenameprefix+filename,
    #                              references=[ref_ip_lqr, ref_ip1, ref_ip2, ref_ip3, ref_ip4],
    #                              labels=['LQR', 'PS: 5', 'PS: 10', 'PS: 20', 'PS: 50'],
    #                              linestyles=['--', '-', '-', '-', '-'])

    # plots.plot_inverted_pendulum(inverted_pendulum=inverted_pendulum,
    #                              animate=animate_plot,
    #                              filename=relative_path+filenameprefix+filename,
    #                              references=[ref_ip_lqr],
    #                              labels=['LQR'],
    #                              linestyles=['--'])

    plots.plot_inverted_pendulum(inverted_pendulum=inverted_pendulum,
                                 animate=animate_plot,
                                 filename=relative_path+filenameprefix+filename)



if __name__ == '__main__':
    main()
