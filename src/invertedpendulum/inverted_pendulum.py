from typing import Callable

import numpy as np

from scipy import integrate
from numpy import pi

from .util import get_value_for_interval
from .engine import IPEngine


class InvertedPendulum(IPEngine):

    def __init__(self, initial_state, desired_position: dict[tuple, float], duration):
        super().__init__()

        # Constraints
        self.max_theta = self.properties.pendulum.max_ang
        self.max_dth = self.properties.pendulum.max_vel
        self.max_ddth = self.properties.pendulum.max_acc
        self.max_f = self.properties.cart.max_force
        self.max_x = self.properties.cart.max_pos
        self.max_dx = self.properties.cart.max_vel
        self.max_ddx = self.properties.cart.max_acc
        self.cart_width = 0.5 * self.L
        self.cart_height = 0.3 * self.cart_width

        # INITIAL STATE
        th = initial_state[0] * pi / 180
        dth = initial_state[1] * pi / 180
        x = initial_state[2]
        dx = initial_state[3]
        self.state_0 = [th, dth, x, dx]

        # DESIRED POSITION
        self.x_ref = desired_position

        # SIMULATION PARAMETERS
        self.t_max = duration
        self.t_eval = np.linspace(0, self.t_max, num=self.t_max * 30)
        self.__t = 0.0
        self.__dt = self.t_eval[1] - self.t_eval[0]
        self.state = np.array(self.state_0, dtype=float).copy()
        self.t = [self.__t]
        self.y = [self.state.copy()]

    def step(self, force_function: Callable):

        # FUNCTION COMPUTING STATE VARIABLES VALUES IN A SINGLE TIMESTAMPS
        def derivs(t_local, state):

            # Create array of zeros of a self.state size
            dsdt = np.zeros_like(state)

            # Assign proper state variables to each state vector indexes
            th, dth, x, dx = state

            # Check the desired position
            x0 = get_value_for_interval(t_local, self.x_ref)

            # Calculate a value of force needed to stabilize the pendulum at a desired x0 position
            F = force_function(th, dth, x - x0, dx)

            # Calculate state variables
            dsdt[0] = dth
            dsdt[1] = self.ddth_func(th, dth, dx, F)
            dsdt[2] = dx
            dsdt[3] = self.ddx_func(th, dth, dx, F)

            return dsdt

        # Solve pendulum's differential equation
        t_next = min(self.__t + self.__dt, self.t_max)
        sol = integrate.solve_ivp(
            derivs,
            method='BDF',
            y0=self.state,
            t_span=(self.__t, t_next),
            max_step=self.__dt
        )
        y_next = sol.y[:, -1]
        t_next = sol.t[-1]

        self.__t = t_next
        self.state = y_next
        self.t.append(t_next)
        self.y.append(y_next)

        return t_next, y_next


    def run(self, force_function: Callable):
        while self.__t < self.t_max:
            self.step(force_function)

        return np.array(self.t), np.stack(self.y, axis=1)


    def reset(self):
        self.__t = 0.0
        self.state = np.array(self.state_0, dtype=float).copy()
        self.t = [self.__t]
        self.y = [self.state.copy()]


    def set_state(self, state):
        th = state[0] * pi / 180
        dth = state[1] * pi / 180
        x = state[2]
        dx = state[3]
        self.state_0 = [th, dth, x, dx]
