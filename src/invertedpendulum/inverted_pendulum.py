from typing import Callable

import numpy as np

from scipy import integrate
from numpy import sin, cos, pi

from invertedpendulum.util import get_value_for_interval
from src.invertedpendulum.physics import InvertedPendulumPhysics


class InvertedPendulum(InvertedPendulumPhysics):

    def __init__(self, initial_state, desired_position: dict[tuple, float], duration):
        super().__init__()

        # Constraints
        self.max_f = self.properties['cart']['max_force']
        self.max_theta = self.properties['pendulum']['max_ang']
        self.max_x = self.properties['cart']['max_pos']
        self.max_dx = self.properties['cart']['max_vel']
        self.max_dth = self.properties['pendulum']['max_vel']
        self.max_ddx = self.properties['cart']['max_acc']
        self.max_ddth = self.properties['pendulum']['max_acc']
        self.cart_width = 0.5 * self.L
        self.cart_height = 0.3 * self.cart_width

        # INITIAL STATE
        self.state = initial_state
        self.state[0] = self.th = initial_state[0] * pi / 180
        self.state[1] = self.dth = initial_state[1] * pi / 180
        self.state[2] = self.x = initial_state[2]
        self.state[3] = self.dx = initial_state[3]

        # DESIRED POSITION
        self.x_ref = desired_position

        # SIMULATION TIME
        self.t_max = duration


    def run(self, force_function: Callable):

        # FUNCTION COMPUTING STATE VARIABLES VALUES IN A SINGLE TIMESTAMPS
        def derivs(t, state):

            # Create array of zeros of a self.state size
            dsdt = np.zeros_like(state)

            # Assign proper state variables to each state vector indexes
            th, dth, x, dx = state

            # Check the desired position
            x0 = get_value_for_interval(t, self.x_ref)

            # Calculate a value of force needed to stabilize the pendulum at a desired x0 position
            F = force_function(th, dth, x - x0, dx)

            # Calculate state variables
            dsdt[0] = dth
            dsdt[1] = self.ddth_func(th, dth, dx, F)
            dsdt[2] = dx
            dsdt[3] = self.ddx_func(th, dth, dx, F)

            return dsdt

        # SIMULATION STOP CONDITION
        def limit(t, state):
            # If pendulum's angle is less than 90 deg, stop simulation
            if abs(state[0]) > 2*pi:
                return 0
            # Otherwise keep performing it
            else:
                return 1

        # If true, terminate integration after event (limit function) occurs
        limit.terminal = True

        # Solve pendulum's differential equation
        sol = integrate.solve_ivp(derivs, y0=self.state, t_span=[0, self.t_max], max_step=1e-1, events=limit)

        return sol.t, sol.y
