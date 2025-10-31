from src.invertedpendulum.inverted_pendulum import InvertedPendulum
from src.invertedpendulum.plotutil import save_solution_plot, save_anim

import numpy as np


def main():

    # ------------------ #
    # INITIAL CONDITIONS #
    # ------------------ #

    th = 30.0      # [deg]
    dth = 100.0     # [deg/s]
    x = 3.0       # [m]
    dx = -6.0     # [m/s]

    # State vector
    state = np.array([th, dth, x, dx])


    # ------------------- #
    # SIMULATION SETTINGS #
    # ------------------- #

    # Time domain
    t_max = 15

    # Desired position
    x_ref = {
        (0, 5): 0,
        (5, 10): 1,
        (10, t_max): -1
    }

    inverted_pendulum = InvertedPendulum(initial_state=state, desired_position=x_ref, duration=t_max)


    # ----------------------------------- #
    #   E X P E R I M E N T   S E T U P   #
    # ----------------------------------- #

    K = inverted_pendulum.compute_lqr(*inverted_pendulum.compute_ss_matrices())

    def force_func(th, dth, x, dx):
        # return 0
        return K[0] * th + K[1] * dth + K[2] * x + K[3] * dx

    t, state = inverted_pendulum.run(force_func)
    save_solution_plot(t, state, x_ref)
    # save_anim(t, state, x_ref)


if __name__ == '__main__':
    main()
