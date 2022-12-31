import itertools

import numpy as np

from scipy import integrate
from numpy import sin, cos, pi


class InvertedPendulum:

    # CONSTRUCTOR
    def __init__(self, pendulum_length, pendulum_mass, pendulum_friction, cart_mass, cart_friction,
                 g, initial_state, desired_position, time_domain):

        # PENDULUM'S PARAMETERS
        self.L = pendulum_length
        self.m = pendulum_mass
        self.b = pendulum_friction
        if hasattr(self.m, '__len__'):
            m = (max(self.m[0])+min(self.m[0]))/2
            self.I = 1 / 3 * m * self.L ** 2
        else:
            self.I = 1 / 3 * self.m * self.L ** 2

        # CART'S PARAMETERS
        self.cart_width = 0.5 * self.L
        self.cart_height = 0.3 * self.cart_width
        self.M = cart_mass
        self.B = cart_friction

        # INITIAL STATE
        self.state = initial_state
        self.state[0] = self.th = initial_state[0] * pi / 180
        self.state[1] = self.dth = initial_state[1] * pi / 180
        self.state[2] = self.x = initial_state[2]
        self.state[3] = self.dx = initial_state[3]

        # DESIRED POSITION
        self.x_ref = desired_position

        # GRAVITY
        self.g = g

        # SIMULATION TIME
        self.t_0 = time_domain[0]
        self.t_max = time_domain[1]
        self.dt = time_domain[2]
        self.t = np.arange(self.t_0, self.t_max, self.dt)

        # CONTROLLER COEFFICIENTS
        self.K = [0, 0, 0, 0]

        # SOLUTION
        self.solution = None

        # X LIMIT FOR VISUALISATION PLOT
        self.xlim = (-1.25*self.L, 1.25*self.L)

    # DEFINE DESIRED CART POSITION(S)
    def step(self, t):

        # Create timestamps array
        timestamps = self.x_ref[0]

        # Create positions array
        positions = self.x_ref[1]

        # Check position corresponding to current time
        for i in range(len(timestamps)):
            if t <= timestamps[i]:
                return positions[i]
            if timestamps[i] < t <= timestamps[i+1]:
                return positions[i+1]
            elif t >= self.t_max:
                return positions[-1]
            else:
                continue

    # DEFINE PENDULUM'S FRICTION
    def friction_step(self, theta):

        # Create timestamps array
        anglestamps = self.b[0]

        # Create positions array
        frictions = self.b[1]

        # Check position corresponding to current time
        for i in range(len(anglestamps)):
            if theta <= anglestamps[i]:
                return frictions[i]
            if anglestamps[i] < theta <= anglestamps[i + 1]:
                return frictions[i+1]
            elif theta >= anglestamps[-1]:
                return frictions[-1]
            else:
                continue

    # DEFINE PENDULUM'S FRICTION
    def cart_friction_step(self, position):

        # Create timestamps array
        positionstamps = self.B[0]

        # Create positions array
        frictions = self.B[1]

        # Check position corresponding to current time
        for i in range(len(positionstamps)):
            if position <= positionstamps[i]:
                return frictions[i]
            if positionstamps[i] < position <= positionstamps[i + 1]:
                return frictions[i+1]
            elif position >= positionstamps[-1]:
                return frictions[-1]
            else:
                continue

    # DEFINE PENDULUM'S FRICTION
    def pendulum_mass_step(self, t):

        # Create timestamps array
        timestamps = self.m[0]

        # Create positions array
        masses = self.m[1]

        # Check position corresponding to current time
        for i in range(len(timestamps)):
            if t <= timestamps[i]:
                return masses[i]
            if timestamps[i] < t <= timestamps[i + 1]:
                return masses[i+1]
            elif t >= timestamps[-1]:
                return masses[-1]
            else:
                continue

    # SOLVE PENDULUM'S DIFFERENTIAL EQUATION AND STORE RESULTS
    def calculate(self):

        # FUNCTION COMPUTING STATE VARIABLES VALUES IN A SINGLE TIMESTAMPS
        def derivs(t, state):

            # Create array of zeros of a self.state size
            dsdt = np.zeros_like(state)

            # Assign proper state variables to each state vector indexes
            th = state[0]
            dth = state[1]
            x = state[2]
            dx = state[3]

            # Check the desired position
            x0 = self.step(t)

            # Assign proper friction
            if hasattr(self.b, '__len__'):
                b = self.friction_step(th*180.0/pi)
            else:
                b = self.b

            if hasattr(self.B, '__len__'):
                B = self.cart_friction_step(x)
            else:
                B = self.B

            if hasattr(self.m, '__len__'):
                m = self.pendulum_mass_step(t)
                self.I = 1 / 3 * m * self.L ** 2
            else:
                m = self.m

            # Calculate a value of force needed to stabilize the pendulum at a desired x0 position
            F = self.K[0] * th + self.K[1] * dth + self.K[2] * (x - x0) + self.K[3] * dx

            # Calculate state variables
            dsdt[0] = dth
            dsdt[1] = (2*B*self.L*dx*m*cos(th) - 2*F*self.L*m*cos(th) - self.L**2*dth**2*m**2*sin(2*th)/2 - 2*self.L*self.M*self.g*m*sin(th) -
                       2*self.L*self.g*m**2*sin(th) - 4*self.M*b*dth - 4*b*dth*m)/(4*self.I*self.M + 4*self.I*m + self.L**2*self.M*m + self.L**2*m**2*sin(th)**2)
            dsdt[2] = dx
            dsdt[3] = (-4*B*self.I*dx - B*self.L**2*dx*m + 4*F*self.I + F*self.L**2*m + 2*self.I*self.L*dth**2*m*sin(th) + self.L**3*dth**2*m**2*sin(th)/2
                       + self.L**2*self.g*m**2*sin(2*th)/2 + 2*self.L*b*dth*m*cos(th))/(4*self.I*self.M + 4*self.I*m + self.L**2*self.M*m + self.L**2*m**2*sin(th)**2)

            return dsdt

        # SIMULATION STOP CONDITION
        def limit(t, state):
            # If pendulum's angle is less than 90 deg, stop simulation
            if abs(state[0]) > 4*pi/2:
                state[0] = 0
                return state[0]
            # Otherwise keep performing it
            else:
                return 1

        # If true, terminate integration after event (limit function) occurs
        limit.terminal = True

        # Solve pendulum's differential equation
        sol = integrate.solve_ivp(derivs, y0=self.state, t_span=[0, self.t_max], max_step=self.dt, events=limit)

        # Store time domain and solution values
        self.t = sol.t
        self.solution = sol.y

        return self.t, self.solution

    # GET ANGULAR POSITION INTEGRAL OVER TIME
    def get_theta_integral(self):

        # Make an absolute out of each value in solved theta array
        ths = [abs(x) for x in self.solution[0]]

        # Initialize integral
        sum = 0
        itheta = []

        # Integrate
        for i in range(len(ths)):
            if i < len(ths)-1:
                dt = self.t[i+1] - self.t[i]

            sum += dt * ths[i]
            itheta.append(sum)

        return sum, itheta

    # GET CART'S POSITION INTEGRAL OVER TIME
    def get_x_integral(self):

        # Get solved positions array
        xs = self.solution[2]

        # Make an absolute out of each value in solved x array and subtracted desired position value
        xs_ref = [abs(xs[i] - self.step(self.t[i])) for i in range(len(xs))]

        # Initialize integral
        sum = 0
        ix = []

        # Integrate
        for i in range(len(xs)):
            if i < len(xs_ref)-1:
                dt = self.t[i+1] - self.t[i]
            sum += dt * xs_ref[i]
            ix.append(sum)

        return sum, ix

    def copy(self):
        return InvertedPendulum(self.L, self.m, self.b, self.M, self.B, self.g,
                                [self.state[0] * 180/pi, self.state[1] * 180/pi, self.state[2], self.state[3]],
                                self.x_ref, (self.t_0, self.t_max, self.dt))
