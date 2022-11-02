import itertools

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Rectangle
from matplotlib import gridspec

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
        # self.K = [41, 19, 1.5, 3.8]
        # self.K = [50, 20, 3.1, 4.8]
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

            # Calculate a value of force needed to stabilize the pendulum at a desired x0 position
            F = self.K[0] * th + self.K[1] * dth + self.K[2] * (x - x0) + self.K[3] * dx

            # Calculate state variables
            dsdt[0] = dth
            dsdt[1] = (2*self.B*self.L*dx*self.m*cos(th) - 2*F*self.L*self.m*cos(th) - self.L**2*dth**2*self.m**2*sin(2*th)/2 - 2*self.L*self.M*self.g*self.m*sin(th) -
                       2*self.L*self.g*self.m**2*sin(th) - 4*self.M*self.b*dth - 4*self.b*dth*self.m)/(4*self.I*self.M + 4*self.I*self.m + self.L**2*self.M*self.m + self.L**2*self.m**2*sin(th)**2)
            dsdt[2] = dx
            dsdt[3] = (-4*self.B*self.I*dx - self.B*self.L**2*dx*self.m + 4*F*self.I + F*self.L**2*self.m + 2*self.I*self.L*dth**2*self.m*sin(th) + self.L**3*dth**2*self.m**2*sin(th)/2
                       + self.L**2*self.g*self.m**2*sin(2*th)/2 + 2*self.L*self.b*dth*self.m*cos(th))/(4*self.I*self.M + 4*self.I*self.m + self.L**2*self.M*self.m + self.L**2*self.m**2*sin(th)**2)

            return dsdt

        # SIMULATION STOP CONDITION
        def limit(t, state):
            # If pendulum's angle is less than 90 deg, stop simulation
            if abs(state[0]) > pi/2:
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

    # GET ANGULAR POSITION INTEGRAL OVER TIME
    def get_theta_integral(self):

        # Make an absolute out of each value in solved theta array
        ths = [abs(x) for x in self.solution[0]]

        # Initialize integral
        sum = 0
        # itheta = []

        # Integrate
        for i in range(len(ths)):
            if i < len(ths)-1:
                dt = self.t[i+1] - self.t[i]

            sum += dt * ths[i]
            # itheta.append(sum)

        # plt.plot(self.t, itheta)
        # plt.grid()
        # plt.draw()

        return sum

    # GET CART'S POSITION INTEGRAL OVER TIME
    def get_x_integral(self):

        # Get solved positions array
        xs = self.solution[2]

        # Make an absolute out of each value in solved x array and subtracted desired position value
        xs_ref = [abs(xs[i] - self.step(self.t[i])) for i in range(len(xs))]

        # Initialize integral
        sum = 0
        # ix = []

        # Integrate
        for i in range(len(xs)):
            if i < len(xs_ref)-1:
                dt = self.t[i+1] - self.t[i]
            sum += dt * xs_ref[i]
            # ix.append(sum)

        # plt.plot(self.t, ix)
        # plt.grid()
        # plt.draw()

        return sum

    # VISUALIZE SIMULATION ON A GRAPH
    def plot(self, animate, save):

        # ASSIGN SELF.SOLUTION AND SELF.L AS LOCAL VARIABLES
        solution = self.solution
        L = self.L

        # GET PENDULUM'S ANGLE
        ths = solution[0]                            # [rad]
        thsdeg = [rad * 180.0 / pi for rad in ths]   # [deg]

        # GET PENDULUM'S ANGULAR VELOCITY
        dths = solution[1]                           # [rad/s]
        dthsdeg = [rad * 180.0 / pi for rad in ths]  # [deg/s]

        # GET PENDULUM'S POSITION
        xs = solution[2]                             # [m]

        # GET PENDULUM'S VELOCITY
        dxs = solution[3]                            # [m/s]

        # CALCULATE PENDULUM'S COORDINATES
        x_pos = L * sin(ths) + xs
        y_pos = L * cos(ths)

        self.xlim = (x_pos[0] - L * 1.25, x_pos[0] + L * 1.25)
        ylim = (-L * 1.25, L * 1.25)

        # IF ANIMATE IS TRUE, VISUALISE PENDULUM'S MOVEMENT IN X-Y COORDINATE SYSTEM
        if animate:

            # DEFINE 4x2 [rows x columns] SUBPLOTS GRID
            gs = gridspec.GridSpec(4, 2, width_ratios=[1, 1],
                                   height_ratios=[1, 1, 1, 1])

            # CREATE A FIGURE
            fig = plt.figure(figsize=(15, 15), dpi=80)

            # CREATE X-Y SUBPLOT WITH VISUALISATION OF INVERTED PENDULUM
            ax = fig.add_subplot(gs[:, 0])
            ax.set_xlim(self.xlim)
            ax.set_ylim(ylim)
            ax.set_aspect('equal')
            ax.grid()
            ax.set_xlabel('x [m]')
            ax.set_ylabel('y [m]')

            # Plot pendulum
            line_pendulum, = ax.plot([], [], 'o-', color='0.3', lw=3)
            # Plot cart
            cart = ax.add_patch(Rectangle((0, 0),
                                          width=self.cart_width,
                                          height=self.cart_height,
                                          facecolor='0.6',
                                          edgecolor='0'))

            # Plot desired position
            x_des_plot, = ax.plot([], [], '--', color='0.3', lw=1)

            # Create text templates
            time_template = 'time = %.1f s'
            time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes)
            theta_template = 'theta = %.1f\N{DEGREE SIGN}'
            theta_text = ax.text(0.3, 0.95, '', transform=ax.transAxes)
            xdes_template = 'x_des = %.1f m'
            xdes_text = ax.text(0.7, 0.95, '', transform=ax.transAxes)

            # CREATE THETA SUBPLOT
            axth = fig.add_subplot(gs[0, 1])
            axth.grid()
            axth.set_xlabel('t [s]')
            axth.set_ylabel('\u03B8 [\N{DEGREE SIGN}]')
            axth.plot(self.t, thsdeg, '-', color='0.3', lw=3)
            # Plot theta reference
            axth.plot([0, self.t_max], [0, 0], '--', color='0.3', lw=1)

            # CREATE POSITION SUBPLOT
            axx = fig.add_subplot(gs[1, 1])
            axx.grid()
            axx.set_xlabel('t [s]')
            axx.set_ylabel('x [m]')
            axx.plot(self.t, xs, '-', color='0.3', lw=3)
            # Plot x reference
            t_ref = [[0]]
            t_ref.extend([[self.x_ref[0][i], self.x_ref[0][i]] for i in range(len(self.x_ref[0])-1)])
            t_ref = list(itertools.chain.from_iterable(t_ref))
            t_ref.append(self.t_max)

            x_ref = []
            x_ref.extend([[self.x_ref[1][i], self.x_ref[1][i]] for i in range(len(self.x_ref[1]))])
            x_ref = list(itertools.chain.from_iterable(x_ref))
            axx.plot(t_ref, x_ref, '--', color='0.3', lw=1)

            # CREATE ANGULAR VELOCITY SUBPLOT
            axdth = fig.add_subplot(gs[2, 1])
            axdth.grid()
            axdth.set_xlabel('t [s]')
            axdth.set_ylabel('\u03C9 [1/s]')
            axdth.plot(self.t, dths, '-', color='0.3', lw=3)

            # CREATE VELOCITY SUBPLOT
            axdx = fig.add_subplot(gs[3, 1])
            axdx.grid()
            axdx.set_xlabel('t [s]')
            axdx.set_ylabel('v [m/s]')
            axdx.plot(self.t, dxs, '-', color='0.3', lw=3)

            # IF PLOT IS ABOUT TO BE SAVED, SET PROPER blit PARAMETER FOR ANIMATION FUNCTION
            if save:
                blit = False
            else:
                blit = True

            # INITIALIZE INVERTED PENDULUM'S X-Y SUBPLOT
            def init():
                line_pendulum.set_data([], [])
                cart.set_xy(([], []))
                time_text.set_text('')
                theta_text.set_text('')
                xdes_text.set_text('')

                return line_pendulum, cart, time_text, theta_text, xdes_text

            # ANIMATE FUNCTION
            def animate(i):
                # GET VARIABLES
                thist = self.t[i]                           # current time
                thisth = ths[i]                             # current theta
                thisx = [xs[i], x_pos[i]]                   # current pendulum's (x, y)
                thisy = [0, y_pos[i]]                       # current cart's (x, y)
                thisdx = dxs[i]                             # current cart's velocity
                thisxdes = self.step(thist)                 # current x_des

                # UPDATE INVERTED PENDULUM'S POSITION
                line_pendulum.set_data(thisx, thisy)
                cart.set_xy((xs[i] - self.cart_width / 2, -self.cart_height / 2))

                # UPDATE INVERTED PENDULUM'S X-Y PLOT LIMIT
                if xs[i] + 1.25*self.cart_width/2 > self.xlim[1]:
                    self.xlim = (xs[i] + 1.25*self.cart_width/2 - 2*L*1.25, xs[i] + 1.25*self.cart_width/2)
                    ax.set_xlim(self.xlim)
                elif xs[i] - 1.25*self.cart_width/2 < self.xlim[0]:
                    self.xlim = (xs[i] - 1.25*self.cart_width/2, xs[i] - 1.25*self.cart_width/2 + 2*L*1.25)
                    ax.set_xlim(self.xlim)

                # UPDATE TEXT LABELS
                time_text.set_text(time_template % thist)
                theta_text.set_text(theta_template % (thisth * 180.0 / pi))
                xdes_text.set_text(xdes_template % thisxdes)

                # UPDATE DESIRED POSITION
                x_des_plot.set_data([thisxdes, thisxdes, thisxdes], [-2*L, 0, 2*L])

                # UPDATE PLOT'S LIMITS
                axth.set_xlim(left=0, right=thist)
                axx.set_xlim(left=0, right=thist)
                axdth.set_xlim(left=0, right=thist)
                axdx.set_xlim(left=0, right=thist)

                return line_pendulum, cart, time_text, theta_text, xdes_text, ax, axth, axx, axdth, axdx, x_des_plot

            # INVOKE ANIMATION FUNCTION
            anim = animation.FuncAnimation(fig, animate, frames=range(1, len(self.t)),
                                          interval=self.t[-1]/len(self.t)*1000, blit=blit, init_func=init)

            # IF SAVE IS TRUE
            if save:
                # SAVE ANIMATION AS GIF
                writergif = animation.PillowWriter(fps=len(self.t)/self.t[-1])
                anim.save("anims/anim.gif", writer=writergif)
            else:
                # SHOW THE PLOT IN A WINDOW
                plt.show()

        # IF ANIMATION IS FALSE
        else:

            # DEFINE 4x1 [rows x columns] SUBPLOTS GRID
            gs = gridspec.GridSpec(4, 1, width_ratios=[1],
                                   height_ratios=[1, 1, 1, 1])

            # CREATE A FIGURE
            fig = plt.figure(figsize=(10, 15), dpi=80)

            # CREATE THETA SUBPLOT
            axth = fig.add_subplot(gs[0, 0])
            axth.grid()
            axth.set_xlim(left=0, right=self.t_max)
            axth.set_xlabel('t [s]')
            axth.set_ylabel('\u03B8 [\N{DEGREE SIGN}]')
            axth.plot(self.t, thsdeg, '-', color='0.3', lw=3)
            # Plot theta reference
            axth.plot([0, self.t_max], [0, 0], '--', color='0.3', lw=1)

            # CREATE POSITION SUBPLOT
            axx = fig.add_subplot(gs[1, 0])
            axx.grid()
            axx.set_xlim(left=0, right=self.t_max)
            axx.set_xlabel('t [s]')
            axx.set_ylabel('x [m]')
            axx.plot(self.t, xs, '-', color='0.3', lw=3)
            # Plot x reference
            t_ref = [[0]]
            t_ref.extend([[self.x_ref[0][i], self.x_ref[0][i]] for i in range(len(self.x_ref[0])-1)])
            t_ref = list(itertools.chain.from_iterable(t_ref))
            t_ref.append(self.t_max)

            x_ref = []
            x_ref.extend([[self.x_ref[1][i], self.x_ref[1][i]] for i in range(len(self.x_ref[1]))])
            x_ref = list(itertools.chain.from_iterable(x_ref))
            axx.plot(t_ref, x_ref, '--', color='0.3', lw=1)

            # CREATE ANGULAR VELOCITY SUBPLOT
            axdth = fig.add_subplot(gs[2, 0])
            axdth.grid()
            axdth.set_xlim(left=0, right=self.t_max)
            axdth.set_xlabel('t [s]')
            axdth.set_ylabel('\u03C9 [1/s]')
            axdth.plot(self.t, dths, '-', color='0.3', lw=3)

            # CREATE VELOCITY SUBPLOT
            axdx = fig.add_subplot(gs[3, 0])
            axdx.grid()
            axdx.set_xlim(left=0, right=self.t_max)
            axdx.set_xlabel('t [s]')
            axdx.set_ylabel('v [m/s]')
            axdx.plot(self.t, dxs, '-', color='0.3', lw=3)

            # IF SAVE IS TRUE
            if save:
                # SAVE FIGURE TO A FILE
                plt.savefig('plots/plot.png')
            else:
                # SHOW THE PLOT IN A WINDOW
                plt.show()
