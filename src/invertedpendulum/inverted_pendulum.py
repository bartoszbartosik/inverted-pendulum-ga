from typing import Callable

import numpy as np

from scipy import integrate
from numpy import pi

from .util import get_value_for_interval
from .physics import InvertedPendulumPhysics


class InvertedPendulum(InvertedPendulumPhysics):

    def __init__(self, initial_state, desired_position: dict[tuple, float], duration):
        super().__init__()

        # Constraints
        self.max_theta = self.properties['pendulum']['max_ang'] * pi / 180
        self.max_dth = self.properties['pendulum']['max_vel']
        self.max_ddth = self.properties['pendulum']['max_acc']
        self.max_f = self.properties['cart']['max_force']
        self.max_x = self.properties['cart']['max_pos']
        self.max_dx = self.properties['cart']['max_vel']
        self.max_ddx = self.properties['cart']['max_acc']
        self.cart_width = 0.5 * self.L
        self.cart_height = 0.3 * self.cart_width

        # INITIAL STATE
        th = initial_state[0] * pi / 180
        dth = initial_state[1] * pi / 180
        x = initial_state[2]
        dx = initial_state[3]
        self.state = [th, dth, x, dx]

        # DESIRED POSITION
        self.x_ref = desired_position

        # SIMULATION TIME
        self.t_max = duration
        self.t_eval = np.linspace(0, self.t_max, num=self.t_max * 30)


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
        def stop_event(t, state):
            # If pendulum's angle is less than 90 deg, stop simulation
            if abs(state[0]) > self.max_theta:
                return 0
            # If pendulum's angular velocity is out of bounds, stop simulation
            elif abs(state[1]) > self.max_dth:
                return 0
            # If cart's position is out of bounds, stop simulation
            elif abs(state[2]) > self.max_x:
                return 0
            # If cart's velocity is out of bounds, stop simulation
            elif abs(state[3]) > self.max_dx:
                return 0
            # Otherwise keep performing it
            else:
                return 1

        # If true, terminate integration after event occurs
        stop_event.terminal = True

        # Solve pendulum's differential equation
        sol = integrate.solve_ivp(derivs, method='BDF', y0=self.state, t_span=[0, self.t_max], t_eval=self.t_eval, events=stop_event)

        return sol.t, sol.y


    def set_state(self, state):
        th = state[0] * pi / 180
        dth = state[1] * pi / 180
        x = state[2]
        dx = state[3]
        self.state = [th, dth, x, dx]
