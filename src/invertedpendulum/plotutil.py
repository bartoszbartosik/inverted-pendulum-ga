from matplotlib import gridspec, pyplot as plt, animation
from matplotlib.patches import Rectangle
from matplotlib.ticker import AutoMinorLocator
from numpy import pi, sin, cos

from invertedpendulum.config import CONFIGS_DIR, EXPERIMENTS_DIR
from invertedpendulum.util import read_config, get_value_for_interval


CONFIG = 'invertedpendulum'
properties = read_config(CONFIGS_DIR / f"{CONFIG}.yaml")[CONFIG]


def save_solution_plot(filename, t, solution, x_ref: dict[tuple, float]):

    # Duration
    t_max = t[-1]

    # Pendulum's angle
    ths = solution[0]                            # [rad]
    thsdeg = [rad * 180.0 / pi for rad in ths]   # [deg]

    # Pendulum's angular velocity
    dths = solution[1]                           # [rad/s]

    # Cart's position
    xs = solution[2]                             # [m]

    # Cart's velocity
    dxs = solution[3]                            # [m/s]

    # Plot settings
    linestyle = '-'
    linewidth = 3
    color = '0.3'

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

    # 4x1 [rows x columns] subplots grid
    gs = gridspec.GridSpec(4, 1, width_ratios=[1],
                           height_ratios=[1, 1, 1, 1])

    fig = plt.figure(figsize=(10, 15), dpi=80)

    axth = fig.add_subplot(gs[0, 0])
    axx = fig.add_subplot(gs[1, 0])
    axdth = fig.add_subplot(gs[2, 0])
    axdx = fig.add_subplot(gs[3, 0])

    __plot_state_variables(axth, axx, axdth, axdx, t, thsdeg, xs, dths, dxs, t_max, x_ref, linestyle, linewidth, color)

    plt.savefig(EXPERIMENTS_DIR / f'{filename}.png')


def save_anim(filename, t, solution, x_ref: dict[tuple, float]):

    # Duration
    t_max = t[-1]

    # Pendulum's angle
    ths = solution[0]                            # [rad]
    thsdeg = [rad * 180.0 / pi for rad in ths]   # [deg]

    # Pendulum's angular velocity
    dths = solution[1]                           # [rad/s]

    # Cart's position
    xs = solution[2]                             # [m]

    # Cart's velocity
    dxs = solution[3]                            # [m/s]

    # Pendulum's coordinates
    L = properties['pendulum']['length']         # [m]
    x_pos = L * sin(ths) + xs
    y_pos = L * cos(ths)

    # Cart's dimensions
    cart_width = 0.5 * L
    cart_height = 0.3 * cart_width

    # Plot settings
    linestyle = '-'
    linewidth = 3
    color = '0.3'

    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

    gs = gridspec.GridSpec(4, 2, width_ratios=[1, 1],
                           height_ratios=[1, 1, 1, 1])

    # Create a figure
    fig = plt.figure(figsize=(15, 15), dpi=80)

    # Create x-y subplot with visualisation of inverted pendulum
    ax = fig.add_subplot(gs[:, 0])
    xlim = (x_pos[0] - L * 1.25, x_pos[0] + L * 1.25)
    ylim = (-L * 1.25, L * 1.25)
    ax.set_xlim(xlim)
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

    axth = fig.add_subplot(gs[0, 1])
    axx = fig.add_subplot(gs[1, 1])
    axdth = fig.add_subplot(gs[2, 1])
    axdx = fig.add_subplot(gs[3, 1])

    __plot_state_variables(axth, axx, axdth, axdx, t, thsdeg, xs, dths, dxs, t_max, x_ref, linestyle, linewidth, color)

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
        nonlocal xlim
        # GET VARIABLES
        thist = t[i]  # current time
        thisth = ths[i]  # current theta
        thisx = [xs[i], x_pos[i]]  # current pendulum's (x, y)
        thisy = [0, y_pos[i]]  # current cart's (x, y)
        thisdx = dxs[i]  # current cart's velocity
        thisxdes = get_value_for_interval(thist, x_ref)  # current x_des

        # UPDATE INVERTED PENDULUM'S POSITION
        line_pendulum.set_data(thisx, thisy)
        cart.set_xy((xs[i] - cart_width / 2, -cart_height / 2))

        # UPDATE INVERTED PENDULUM'S X-Y PLOT LIMIT
        if xs[i] + 1.25 * cart_width / 2 > xlim[1]:
            xlim = (xs[i] + 1.25 * cart_width / 2 - 2 * L * 1.25, xs[i] + 1.25 * cart_width / 2)
            ax.set_xlim(xlim)
        elif xs[i] - 1.25 * cart_width / 2 < xlim[0]:
            xlim = (xs[i] - 1.25 * cart_width / 2, xs[i] - 1.25 * cart_width / 2 + 2 * L * 1.25)
            ax.set_xlim(xlim)

        # UPDATE TEXT LABELS
        time_text.set_text(time_template % thist)
        theta_text.set_text(theta_template % (thisth * 180.0 / pi))
        xdes_text.set_text(xdes_template % thisxdes)

        # UPDATE DESIRED POSITION
        x_des_plot.set_data([thisxdes, thisxdes, thisxdes], [-2 * L, 0, 2 * L])

        # UPDATE PLOT'S LIMITS
        axth.set_xlim(left=0, right=thist)
        axx.set_xlim(left=0, right=thist)
        axdth.set_xlim(left=0, right=thist)
        axdx.set_xlim(left=0, right=thist)

        return line_pendulum, cart, time_text, theta_text, xdes_text, ax, axth, axx, axdth, axdx, x_des_plot

    # INVOKE ANIMATION FUNCTION
    anim = animation.FuncAnimation(fig, animate, frames=range(1, len(t)),
                                   interval=t[-1] / len(t) * 1000, blit=False, init_func=init)

    # SAVE ANIMATION AS GIF
    writergif = animation.PillowWriter(fps=len(t) / t_max)
    anim.save(EXPERIMENTS_DIR / f'{filename}.gif', writer=writergif)


def __plot_state_variables(axth, axx, axdth, axdx, t, thsdeg, xs, dths, dxs, t_max, x_ref, linestyle, linewidth, color):

    # Angle
    axth.grid()
    axth.set_xlim(left=0, right=t_max)
    axth.set_xlabel('t [s]')
    axth.set_ylabel('\u03B8 [\N{DEGREE SIGN}]')
    axth.plot(t, thsdeg, ls=linestyle, color=color, lw=linewidth)
    # Plot theta reference
    axth.plot([0, t_max], [0, 0], '--', color='0.3', lw=1, label='reference')
    axth.legend()

    # Position
    axx.grid()
    axx.set_xlim(left=0, right=t_max)
    axx.set_xlabel('t [s]')
    axx.set_ylabel('x [m]')
    axx.plot(t, xs, ls=linestyle, color=color, lw=linewidth)
    # Plot x reference
    t_ref = [t for interval in x_ref.keys() for t in interval]
    x_reff = [list(x_ref.values())[0]]+[pos for sublist in [list(x_ref.values())[i:i+2] for i, pos in enumerate(x_ref.values())] for pos in sublist]
    axx.plot(t_ref, x_reff, '--', color='0.3', lw=1, label='reference')
    axx.legend()

    # Angular velocity
    axdth.grid()
    axdth.set_xlim(left=0, right=t_max)
    axdth.set_xlabel('t [s]')
    axdth.set_ylabel('\u03C9 [1/s]')
    axdth.plot(t, dths, ls=linestyle, color=color, lw=linewidth)

    # Velocity
    axdx.grid()
    axdx.set_xlim(left=0, right=t_max)
    axdx.set_xlabel('t [s]')
    axdx.set_ylabel('v [m/s]')
    axdx.plot(t, dxs, ls=linestyle, color=color, lw=linewidth)


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
    plt.savefig('resources/plots/{}-x_integral.png'.format(filename))


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
    plt.savefig('resources/plots/{}-theta_integral.png'.format(filename))
