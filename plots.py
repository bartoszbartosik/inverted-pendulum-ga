import itertools

from matplotlib import gridspec, pyplot as plt, animation
from matplotlib.patches import Rectangle
from matplotlib.ticker import AutoMinorLocator
from numpy import sin, cos, pi





class Plots:

    #####################################
    # CONSTRUCTOR WITH GLOBAL VARIABLES #
    #####################################
    def __init__(self):
        self.xlim = (-1, 1)

    ##########################
    # INVERTED PENDULUM PLOT #
    ##########################
    def plot_inverted_pendulum(self, inverted_pendulum,
                               animate, filename=None, references=None, labels=None, colors=None ,linestyles=None):

        #####################################
        # HANDLE INVERTED PENDULUM VARIABLE #
        #####################################

        # FETCH INVERTED PENDULUM'S PROPERTIES
        t, solution = inverted_pendulum.calculate()
        L = inverted_pendulum.L
        cart_width = inverted_pendulum.cart_width
        cart_height = inverted_pendulum.cart_height

        t_max = t[-1]
        x_ref = inverted_pendulum.x_ref
        step = inverted_pendulum.step

        # GET PENDULUM'S ANGLE
        ths = solution[0]                            # [rad]
        thsdeg = [rad * 180.0 / pi for rad in ths]   # [deg]

        # GET PENDULUM'S ANGULAR VELOCITY
        dths = solution[1]                           # [rad/s]

        # GET PENDULUM'S POSITION
        xs = solution[2]                             # [m]

        # GET PENDULUM'S VELOCITY
        dxs = solution[3]                            # [m/s]

        # CALCULATE PENDULUM'S COORDINATES
        x_pos = L * sin(ths) + xs
        y_pos = L * cos(ths)

        # INITIALIZE AXES LIMITS
        self.xlim = (x_pos[0] - L * 1.25, x_pos[0] + L * 1.25)
        ylim = (-L * 1.25, L * 1.25)

        # SET LINE'S APPEARANCE
        linestyle = '-'
        linewidth = 3
        color = '0.3'
        label=''

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

        # DEFINE 4x1 [rows x columns] SUBPLOTS GRID
        gs = gridspec.GridSpec(4, 1, width_ratios=[1],
                               height_ratios=[1, 1, 1, 1])

        # CREATE A FIGURE
        fig = plt.figure(figsize=(10, 15), dpi=80)

        # CREATE THETA SUBPLOT
        axth = fig.add_subplot(gs[0, 0])
        axth.grid()
        axth.set_xlim(left=0, right=t_max)
        axth.set_xlabel('t [s]')
        axth.set_ylabel('\u03B8 [\N{DEGREE SIGN}]')
        axth.plot(t, thsdeg, ls=linestyle, color=color, lw=linewidth, label=label)
        axth.legend()
        # Plot theta reference
        axth.plot([0, t_max], [0, 0], '--', color='0.3', lw=1, label="reference")
        axth.legend()

        # CREATE POSITION SUBPLOT
        axx = fig.add_subplot(gs[1, 0])
        axx.grid()
        axx.set_xlim(left=0, right=t_max)
        axx.set_xlabel('t [s]')
        axx.set_ylabel('x [m]')
        axx.plot(t, xs, ls=linestyle, color=color, lw=linewidth, label=label)
        # Plot x reference
        t_ref = [[0]]
        t_ref.extend([[x_ref[0][i], x_ref[0][i]] for i in range(len(x_ref[0]) - 1)])
        t_ref = list(itertools.chain.from_iterable(t_ref))
        t_ref.append(t_max)

        x_reff = []
        x_reff.extend([[x_ref[1][i], x_ref[1][i]] for i in range(len(x_ref[1]))])
        x_reff = list(itertools.chain.from_iterable(x_reff))
        axx.plot(t_ref, x_reff, '--', color='0.3', lw=1, label="reference")
        axx.legend()

        # CREATE ANGULAR VELOCITY SUBPLOT
        axdth = fig.add_subplot(gs[2, 0])
        axdth.grid()
        axdth.set_xlim(left=0, right=t_max)
        axdth.set_xlabel('t [s]')
        axdth.set_ylabel('\u03C9 [1/s]')
        axdth.plot(t, dths, ls=linestyle, color=color, lw=linewidth, label=label)
        axdth.legend()

        # CREATE VELOCITY SUBPLOT
        axdx = fig.add_subplot(gs[3, 0])
        axdx.grid()
        axdx.set_xlim(left=0, right=t_max)
        axdx.set_xlabel('t [s]')
        axdx.set_ylabel('v [m/s]')
        axdx.plot(t, dxs, ls=linestyle, color=color, lw=linewidth, label=label)
        axdx.legend()

        # IF REFERENCE INVERTED PENDULUM(S) IS(ARE) GIVEN
        if references:
            if colors is None:
                colors = ['r', 'b', 'g', 'c', 'm', 'y']

            if labels is None:
                labels = ["ref"] * len(references)

            if linestyles is None:
                linestyles = ['--']*len(references)

            for i in range(len(references)):
                ref_t, ref_solution = references[i].calculate()

                # GET PENDULUM'S ANGLE
                ref_ths = ref_solution[0]  # [rad]
                ref_thsdeg = [rad * 180.0 / pi for rad in ref_ths]  # [deg]

                # GET PENDULUM'S ANGULAR VELOCITY
                ref_dths = ref_solution[1]  # [rad/s]

                # GET PENDULUM'S POSITION
                ref_xs = ref_solution[2]  # [m]

                # GET PENDULUM'S VELOCITY
                ref_dxs = ref_solution[3]  # [m/s]

                axth.plot(ref_t, ref_thsdeg, ls=linestyles[i], color=colors[i], lw=1.5, label=labels[i])
                axth.legend()
                axdth.plot(ref_t, ref_dths, ls=linestyles[i], color=colors[i], lw=1.5, label=labels[i])
                axdth.legend()
                axx.plot(ref_t, ref_xs, ls=linestyles[i], color=colors[i], lw=1.5, label=labels[i])
                axx.legend()
                axdx.plot(ref_t, ref_dxs, ls=linestyles[i], color=colors[i], lw=1.5, label=labels[i])
                axdx.legend()

        # IF FILE NAME IS NONE
        if filename is None:
            filename = "plot"
        # SAVE FIGURE TO A FILE
        plt.savefig('plots/{}.png'.format(filename))

        # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

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
                                          width=cart_width,
                                          height=cart_height,
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
            axth.plot(t, thsdeg, '-', color='0.3', lw=3)
            # Plot theta reference
            axth.plot([0, t_max], [0, 0], '--', color='0.3', lw=1)

            # CREATE POSITION SUBPLOT
            axx = fig.add_subplot(gs[1, 1])
            axx.grid()
            axx.set_xlabel('t [s]')
            axx.set_ylabel('x [m]')
            axx.plot(t, xs, '-', color='0.3', lw=3)
            # Plot x reference
            t_ref = [[0]]
            t_ref.extend([[x_ref[0][i], x_ref[0][i]] for i in range(len(x_ref[0])-1)])
            t_ref = list(itertools.chain.from_iterable(t_ref))
            t_ref.append(t_max)

            x_reff = []
            x_reff.extend([[x_ref[1][i], x_ref[1][i]] for i in range(len(x_ref[1]))])
            x_reff = list(itertools.chain.from_iterable(x_reff))
            axx.plot(t_ref, x_reff, '--', color='0.3', lw=1)

            # CREATE ANGULAR VELOCITY SUBPLOT
            axdth = fig.add_subplot(gs[2, 1])
            axdth.grid()
            axdth.set_xlabel('t [s]')
            axdth.set_ylabel('\u03C9 [1/s]')
            axdth.plot(t, dths, '-', color='0.3', lw=3)

            # CREATE VELOCITY SUBPLOT
            axdx = fig.add_subplot(gs[3, 1])
            axdx.grid()
            axdx.set_xlabel('t [s]')
            axdx.set_ylabel('v [m/s]')
            axdx.plot(t, dxs, '-', color='0.3', lw=3)

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
                thist = t[i]                           # current time
                thisth = ths[i]                             # current theta
                thisx = [xs[i], x_pos[i]]                   # current pendulum's (x, y)
                thisy = [0, y_pos[i]]                       # current cart's (x, y)
                thisdx = dxs[i]                             # current cart's velocity
                thisxdes = step(thist)                 # current x_des

                # UPDATE INVERTED PENDULUM'S POSITION
                line_pendulum.set_data(thisx, thisy)
                cart.set_xy((xs[i] - cart_width / 2, -cart_height / 2))

                # UPDATE INVERTED PENDULUM'S X-Y PLOT LIMIT
                if xs[i] + 1.25*cart_width/2 > self.xlim[1]:
                    self.xlim = (xs[i] + 1.25*cart_width/2 - 2*L*1.25, xs[i] + 1.25*cart_width/2)
                    ax.set_xlim(self.xlim)
                elif xs[i] - 1.25*cart_width/2 < self.xlim[0]:
                    self.xlim = (xs[i] - 1.25*cart_width/2, xs[i] - 1.25*cart_width/2 + 2*L*1.25)
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
            anim = animation.FuncAnimation(fig, animate, frames=range(1, len(t)),
                                          interval=t[-1]/len(t)*1000, blit=False, init_func=init)

            # IF FILE NAME IS NONE
            if filename is None:
                filename = "anim"

            # SAVE ANIMATION AS GIF
            writergif = animation.PillowWriter(fps=len(t)/t_max)
            anim.save("anims/{}.gif".format(filename), writer=writergif)

    def plot_ga_generations(self, geneticalgorithm, filename):
        iterations = geneticalgorithm.iterations
        values = geneticalgorithm.values

        fig, ax = plt.subplots()

        ax.plot(iterations, values, '-', color='0.3', lw=3)

        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_minor_locator(AutoMinorLocator())

        ax.set_xlim(0, iterations[-1])
        ax.set_ylim(min(values), max(values))
        ax.set_xlabel('GENERATION')
        ax.set_ylabel('FITNESS')
        ax.grid(which='major')
        ax.grid(which='minor', lw='0.3')
        plt.savefig('plots/{}-generations.png'.format(filename))

    def plot_pendulum_friction(self, inverted_pendulum, filename):

        b = inverted_pendulum.b

        fig, ax = plt.subplots()

        th_ext = [[-180]]
        th_ext.extend([[b[0][i], b[0][i]] for i in range(len(b[0]) - 1)])
        th_ext = list(itertools.chain.from_iterable(th_ext))
        th_ext.append(b[0][-1])

        b_ext = []
        b_ext.extend([[b[1][i], b[1][i]] for i in range(len(b[1]))])
        b_ext = list(itertools.chain.from_iterable(b_ext))

        ax.plot(th_ext, b_ext, '-', color='0.3', lw=2)

        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.set_xlim(-30, 30)
        ax.set_ylim(0, max(b_ext)*1.2)
        ax.set_xlabel('\u03B8 [\N{DEGREE SIGN}]')
        ax.set_ylabel('b [-]')
        ax.grid(which='major')
        ax.grid(which='minor', lw='0.3')
        plt.savefig('plots/{}-friction.png'.format(filename))

    def plot_cart_friction(self, inverted_pendulum, filename):

        B = inverted_pendulum.B

        fig, ax = plt.subplots()

        x_ext = [[B[0][0]]]
        x_ext.extend([[B[0][i], B[0][i]] for i in range(len(B[0]) - 1)])
        x_ext = list(itertools.chain.from_iterable(x_ext))
        x_ext.append(B[0][-1])

        B_ext = []
        B_ext.extend([[B[1][i], B[1][i]] for i in range(len(B[1]))])
        B_ext = list(itertools.chain.from_iterable(B_ext))

        ax.plot(x_ext, B_ext, '-', color='0.3', lw=2)

        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.set_xlim(x_ext[0], x_ext[-1])
        ax.set_ylim(0, max(B_ext)*1.2)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('B [-]')
        ax.grid(which='major')
        ax.grid(which='minor', lw='0.3')
        plt.savefig('plots/{}-cart_friction.png'.format(filename))

    def plot_pendulum_mass(self, inverted_pendulum, filename):

        m = inverted_pendulum.m

        fig, ax = plt.subplots()

        t_ext = [[m[0][0]]]
        t_ext.extend([[m[0][i], m[0][i]] for i in range(len(m[0]) - 1)])
        t_ext = list(itertools.chain.from_iterable(t_ext))
        t_ext.append(m[0][-1])

        m_ext = []
        m_ext.extend([[m[1][i], m[1][i]] for i in range(len(m[1]))])
        m_ext = list(itertools.chain.from_iterable(m_ext))

        ax.plot(t_ext, m_ext, '-', color='0.3', lw=2)

        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.set_xlim(t_ext[0], t_ext[-1])
        ax.set_ylim(0, max(m_ext)*1.2)
        ax.set_xlabel('t [s]')
        ax.set_ylabel('m [kg]')
        ax.grid(which='major')
        ax.grid(which='minor', lw='0.3')
        plt.savefig('plots/{}-pendulum_mass.png'.format(filename))

    def plot_x_integral(self, inverted_pendulum, filename):

        _, ix = inverted_pendulum.get_x_integral()
        t = inverted_pendulum.t

        fig, ax = plt.subplots()

        ax.plot(t, ix, '-', color='0.3', lw=2)

        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(min(ix), max(ix)*1.2)
        ax.set_xlabel('t [s]')
        ax.set_ylabel('X [m\u22C5s]')
        ax.grid(which='major')
        ax.grid(which='minor', lw='0.3')
        plt.savefig('plots/{}-x_integral.png'.format(filename))

    def plot_theta_integral(self, inverted_pendulum, filename):

        _, itheta = inverted_pendulum.get_theta_integral()
        t = inverted_pendulum.t

        fig, ax = plt.subplots()

        ax.plot(t, itheta, '-', color='0.3', lw=2)

        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.set_xlim(t[0], t[-1])
        ax.set_ylim(min(itheta), max(itheta)*1.2)
        ax.set_xlabel('t [s]')
        ax.set_ylabel('\u03F4 [rad\u22C5s]')
        ax.grid(which='major')
        ax.grid(which='minor', lw='0.3')
        plt.savefig('plots/{}-theta_integral.png'.format(filename))
