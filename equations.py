from enum import Enum

import numpy as np
import sympy
from matplotlib import pyplot as plt
from sympy import symbols
from sympy import sin, cos
from sympy.parsing.sympy_parser import parse_expr

import control as ctrl



def main():
    # Equation 1
    # (I + m * L ^ 2 / 4) * ddth + m * g * L / 2 * sin(th) + b * dth = -m * L / 2 * ddx * cos(th)
    # (I + m*L**2/4)*FI*s**2 + m*g*L/2*th + b*FI*s = -m*L/2*X*s**2
    # Equation 2
    # (m + M) * ddx + B * dx + m * L / 2 * ddth * cos(th) - m * L / 2 * dth ^ 2 * sin(th) = F
    # (m + M)*X*s**2 + B*X*s + m*L/2*FI*s**2 = F

    symbolic = True
    linearize = True

    if symbolic:
        L, m, b, B, M, g, I = symbols('L m b B M g I')
    else:
        # Pendulum
        L = 0.8  # [m]
        m = 0.3  # [kg]
        b = 0.05  # [kg/s]

        # Cart
        M = 0.6  # [kg]
        B = 0.1  # [1/s]

        # Gravitational acceleration
        g = -9.81  # [m/s^2]
        I = 1 / 3 * m * L ** 2

    th, dth, dx, ddth, ddx = symbols('th dth dx ddth ddx')
    F = symbols('F')

    sol = sympy.solve([
        (I + m * L ** 2 / 4) * ddth + m * g * L / 2 * sin(th) + b * dth + m * L / 2 * ddx * cos(th),    # EQUATION 1
        (m + M) * ddx + B * dx + m * L / 2 * ddth * cos(th) - m * L / 2 * dth ** 2 * sin(th) - F        # EQUATION 2
        ],
        ddth, ddx)

    ddth = sympy.simplify(sol.get(ddth))
    ddx = sympy.simplify(sol.get(ddx))
    print('ddtheta: {}'.format(ddth))
    print('ddx: {}'.format(ddx))

    if linearize:
        ddth = parse_expr(str(ddth).\
            replace('cos(th)', '1').replace('dth**2', '0').replace('sin(2*th)', '2*th').replace('sin(th)', 'th'))
        ddx = parse_expr(str(ddx).\
            replace('cos(th)', '1').replace('dth**2', '0').replace('sin(2*th)', '2*th').replace('sin(th)', 'th'))

        print('ddtheta: {}'.format(ddth))
        print('ddx: {}'.format(ddx))

        a21 = float(ddth.coeff(th))
        a22 = float(ddth.coeff(dth))
        a24 = float(ddth.coeff(dx))
        a41 = float(ddx.coeff(th))
        a42 = float(ddx.coeff(dth))
        a44 = float(ddx.coeff(dx))

        b2 = float(ddth.coeff(F))
        b4 = float(ddx.coeff(F))

        # x1 = dx
        # x2 = ddx
        # x3 = dth
        # x4 = ddth

        # [[dth], [ddth], [dx], [ddx]] = A*[[th], [dth], [x], [dx]] + B*u
        # y = C*[[th], [dth], [x], [dx]] + D*u
        #  dth           th
        #  ddth          dth
        #  dx  =  A *    x      +   B * U
        #  ddx           dx

        A = np.matrix([
          # [th,    dth,   x,     dx]
            [0,     1,      0,      0],     # dth
            [a21,   a22,    0,    a24],     # ddth
            [0,     0,      0,      1],     # dx
            [a41,   a42,    0,    a44]      # ddx
        ])

        B = np.matrix([
            [0],
            [b2],
            [0],
            [b4]
        ])

        C = np.matrix([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
        ])

        D = np.matrix([
            [0],
            [0]
        ])

        ss = ctrl.ss(A, B, C, D,
                     states=['th', 'dth', 'x', 'dx'],
                     inputs=['F'],
                     outputs=['th', 'x'])

        Q = np.diag([2800, 0, 100, 0])
        R = 1
        K, S, E = ctrl.lqr(ss, Q, R)
        print(ss)
        print(abs(K))

        ss_closed = ctrl.ss(A - B * K, B, C, D,
                     states=['th', 'dth', 'x', 'dx'],
                     inputs=['F'],
                     outputs=['th', 'x'])

        T, yout = ctrl.step_response(ss_closed, T=np.arange(0, 5, 0.01))

        plt.rcParams["figure.figsize"] = (12, 8)
        fig, (ax1, ax2) = plt.subplots(2)
        ax1.plot(T, yout[0, :].T)
        ax2.plot(T, yout[1, :].T)
        ax1.set_xlabel('Time')
        ax1.set_ylabel('Angle')
        ax1.grid(which='both')
        ax2.set_xlabel('Time')
        ax2.set_ylabel('Position')
        ax2.grid(which='both')

        plt.show()

if __name__ == '__main__':
    main()