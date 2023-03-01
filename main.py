from invertedpendulum.inverted_pendulum import InvertedPendulum
from geneticalgorithm.geneticalgorithm import GeneticAlgorithm
from geneticalgorithm.geneticalgorithm import StopCondition
from invertedpendulum.lqr4ip import LQR4IP
from plots import Plots

import numpy as np


def main():

    ###################################################################################################################
    ###############################    I N V E R T E D   P E N D U L U M   S E T U P    ###############################

    ##########################
    ### PHYSICAL VARIABLES ###
    ##########################
    b_v1 = [
        [-180, -5, 1, 180],     # angle [deg]
        [0.05, 0.05, 0.075, 0.05]          # friction [-]
    ]

    b_v2 = [
        [-180, -5, 1, 180],     # angle [deg]
        [0.05, 0.05, 0.025, 0.05]          # friction [-]
    ]

    B_v1 = [
        [-2, -1.2, -0.8, 0, 1, 1.5, 5],     # position [m]
        [0.1, 0.1, 0.05, 0.08, 0.1, 0.15, 0.1]          # friction [-]
    ]

    B_v2 = [
        [-2, -1.1, -0.9, -0.2, 0, 1, 1.2, 5],     # position [m]
        [0.1, 0.1, 0.15, 0.1, 0.2, 0.1, 0.02, 0.1]          # friction [-]
    ]

    m_v1 = [
        [1, 3, 5, 8, 12, 15], # time [s]
        [0.3, 0.25, 0.2, 0.3, 0.35, 0.3]  # mass [kg]
    ]

    m_v2 = [
        [2, 4, 8, 12, 15], # time [s]
        [0.2, 0.3, 0.1, 0.5, 0.3]  # mass [kg]
    ]

    ##########################
    ### PHYSICAL CONSTANTS ###
    ##########################
    # Pendulum
    L = 0.8     # [m]
    m = 0.3     # [kg]
    b = 0.05     # [kg/s]

    # Cart
    M = 0.6     # [kg]
    B = 0.1     # [1/s]


    # Gravitational acceleration
    g = -9.81   # [m/s^2]

    ##########################
    ### INITIAL CONDITIONS ###
    ##########################
    th = 30.0      # [deg]
    dth = 100.0     # [deg/s]
    x = 3.0       # [m]
    dx = -2.0     # [m/s]

    # State vector
    state = np.array([th, dth, x, dx])

    ##########################
    ### SIMULATION SETTING ###
    ##########################
    # Time domain
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

    ######################
    ### INITIALIZATION ###
    ######################
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

    ###################################################################################################################
    ######################################    S I M U L A T I O N   S E T U P    ######################################

    ##########################
    ### RESULTS GENERATION ###
    ##########################
    # Generate simulation animation
    animate_plot = True

    # Define path inside the "results/{anims&plots}/"
    relative_path = "controller design/ga"

    # Define filename suffix
    filename = "-test"

    # Filename prefix consisting of initial state conditions
    filenameprefix = "/state[{};{};{};{}]-".format(th, dth, x, dx)

    ############################
    ### CONTROLLER SELECTION ###
    ############################
    # Choose either 'LQR' or 'GA'
    controller = 'GA'

    ###################################################################################################################
    #############################   L I N E A R   Q U A D R A T I C   R E G U L A T O R   #############################

    ###############
    ### WEIGHTS ###
    ###############
    # Q matrix definition
    # Q = [
    #   [   th,    0,      0,      0    ],
    #   [   0,     dth,    0,      0    ],
    #   [   0,     0,      x,      0    ],
    #   [   0,     0,      0,      dx   ]
    # ]
    Q = np.diag([1, 100, 22500, 5000])

    # R matrix definition
    R = 1

    ######################
    ### INITIALIZATION ###
    ######################
    lqr = LQR4IP(inverted_pendulum, Q, R)

    ############################
    ### REFERENCE CONTROLLER ###
    ############################
    # Assign computations result as a reference solution
    K_ref = lqr.K

    if controller.__eq__('LQR'):
        #######
        # SET #
        K_sol = K_ref

        ####################
        # PLOT NAME PREFIX #
        filenameprefix += "lqr[{};{};{};{}]-k[{};{};{};{}]"\
                              .format(str(Q[0][0]), str(Q[1][1]), str(Q[2][2]), str(Q[3][3]),
                                      round(K_sol[0], 2), round(K_sol[1], 2), round(K_sol[2], 2), round(K_sol[3], 2))

    ###################################################################################################################
    ########################### G E N E T I C   A L G O R I T H M   C O N T R O L L E R ###############################
    elif controller.__eq__('GA'):

        ##########################
        ### OBJECTIVE FUNCTION ###
        ##########################
        def objfunction(K):

            # Assign new controller K value
            inverted_pendulum.K = K

            # Perform simulation for controller K defined above
            inverted_pendulum.calculate()

            #######################
            # PERFORMANCE INDEXES #
            # Simulation time
            correction_coeff = inverted_pendulum.t[-1]**2

            # Angle integral over time
            # theta_integral = inverted_pendulum.get_theta_integral()
            theta_integral = 1

            # Position integral over time
            x_integral, _ = inverted_pendulum.get_x_integral()
            # x_integral = 1

            # Return combination of above performance indexes
            return correction_coeff*(1/(theta_integral*x_integral))

        ######################
        ### INITIALIZATION ###
        ######################
        ga = GeneticAlgorithm(objective_function=objfunction,
                              population_size=5,
                              chromosome_size=4,
                              gene_bounds=(0, 100.0),
                              mutation_probability=0.2,
                              crossover_probability=0.4,
                              crossover_rate=0.2)

        # Start the genetic algorithm computations
        K_sol = ga.calculate(StopCondition.ITERATIONS, 100)

        # Plot name prefix containing chosen genetic algorithm's parameters and computed controller's K values
        filenameprefix += "ga[psize{};mprob{};crossprob{};crossrate{};glower{};gupper{}]-k[{};{};{};{}]" \
            .format(ga.population.psize, ga.population.mprobability,
                    ga.population.crossprobability, ga.population.crossrate,
                    ga.population.gbounds[0], ga.population.gbounds[1],
                    round(K_sol[0], 2), round(K_sol[1], 2), round(K_sol[2], 2), round(K_sol[3], 2))

        # Visualize evolution process
        plots = Plots()
        plots.plot_ga_generations(geneticalgorithm=ga,
                                  filename=relative_path + filenameprefix + filename)
    ###################################################################################################################
    ##############################   S E T   O F   C O N S I D E R E D   S C E N A R I O S   ##########################
    else:
        ################
        # NO REGULATOR #
        ################
        K_sol = [0, 0, 0, 0]

        ########################
        ### PLOT NAME PREFIX ###
        ########################
        filenameprefix += "nocontrol"

    #######################################################################
    ### INFLUENCE OF GENETIC ALGORITHM PARAMETERS CHANGE ON THE OUTCOME ###
    #######################################################################

    # VAR POPULATION SIZE #
    K_PS_5 =    [74.75, 24.76, 4.07, 14.5]
    K_PS_10 =   [85.15, 70.88, 15.06, 35.68]
    K_PS_20 =   [89.94, 20.16, 7.92, 13.65]
    K_PS_50 =   [55.69, 13.62, 5.36, 7.6]

    # VAR OBJECTIVE FUNCTION #
    K_TH            = [96.71, 24.68, -0.27, 17.62]
    K_X             = [-6.94, 10.53, -94.52, -64.39]
    K_X_TH_DXLIM    = [91.86, 46.94, 7.79, 28.22]
    K_X_TH_THLIM    = [99.51, 74.77, 11.75, 46.67]

    # VAR MUTATION PROBABILITY #
    K_M001 = [98.73, 81.12, 6.26, 29.78]
    K_M003 = [70.18, 18.5, 6.29, 10.75]
    K_M006 = [87.05, 95.32, 14.85, 44.72]
    K_M01 =  [77.34, 20.51, 7.02, 12.05]
    K_M02 =  [89.94, 20.16, 7.92, 13.65]
    K_M04 =  [99.66, 82.03, 10.92, 18.36]
    K_M08 =  [65.91, 20.02, 8.3, 11.21]

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

    # VAR GENE BOUNDARIES #
    K_100_100_2 = [89.69, 18.35, 4.98, 11.29]
    K_0_100_0   = [96.0, 40.0, 13.0, 22.0]
    K_0_100_1   = [97.3, 47.5, 15.6, 21.6]
    K_0_100_2   = [97.0, 53.04, 16.08, 28.23]
    K_0_1000_0  = [776.0, 342.0, 165.0, 220.0]


    #######################################################################
    ### INFLUENCE OF INVERTED PENDULUM'S NON-LINEARITIES ON THE OUTCOME ###
    #######################################################################

    # VAR PENDULUM'S FRICTION #
    K_b_v1 = [93.91, 26.27, 12.98, 16.62]
    K_b_v2 = [96.25, 52.28, 18.12, 19.27]

    # VAR CART'S FRICTION #
    K_B_v1 = [99.29, 33.97, 16.1, 19.13]
    K_B_v2 = [84.82, 26.07, 12.7, 15.06]

    # VAR PENDULUM'S MASS #
    K_m_v1 = [78.96, 32.22, 7.36, 17.07]
    K_m_v2 = [92.49, 29.5, 9.35, 16.26]


    ##########################################################################
    ### THE BEST OBTAINED CONTROLLER WITH THE USE OF THE GENETIC ALGORITHM ###
    ##########################################################################

    # 1000 ITERATIONS #
    K_BEST = [972.0, 294.0, 214.0, 201.0]

    ###################################################################################################################
    ###############################################   S I M U L A T I O N   ###########################################

    ##########################
    ### REFERENCE SOLUTION ###
    ##########################
    # Create identical comparable inverted pendulum model
    ref_ip_lqr = inverted_pendulum.copy()

    # Control it with a LQR reference controller
    ref_ip_lqr.K = K_ref

    ####################
    ### PLOT RESULTS ###
    ####################
    plots = Plots()

    # Plot both target simulation and the reference solution
    plots.plot_inverted_pendulum(inverted_pendulum=inverted_pendulum,
                                 animate=animate_plot,
                                 filename=relative_path+filenameprefix+filename,
                                 references=[ref_ip_lqr],
                                 labels=['LQR'],
                                 linestyles=['--'])


if __name__ == '__main__':
    main()
