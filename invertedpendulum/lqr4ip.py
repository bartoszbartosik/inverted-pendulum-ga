import numpy as np
import sympy
from sympy import symbols, parse_expr
from sympy import sin, cos
import control as ctrl


class LQR4IP:

    def __init__(self, inverted_pendulum, Q, R):
        # Pendulum
        L = inverted_pendulum.L
        m = inverted_pendulum.m
        b = inverted_pendulum.b

        if hasattr(b, '__len__'):
            b = (max(b[0])+min(b[0]))/2


        if hasattr(m, '__len__'):
            m = (max(m[0])+min(m[0]))/2

        # Cart
        M = inverted_pendulum.M
        B = inverted_pendulum.B

        if hasattr(B, '__len__'):
            B = (max(B[0])+min(B[0]))/2

        # Gravitational acceleration
        g = inverted_pendulum.g

        I = 1 / 3 * m * L ** 2

        # CONVERT TO STATE-SPACE
        th, dth, dx, ddth, ddx = symbols('th dth dx ddth ddx')
        F = symbols('F')

        sol = sympy.solve([
            (I + m * L ** 2 / 4) * ddth + m * g * L / 2 * sin(th) + b * dth + m * L / 2 * ddx * cos(th),  # EQUATION 1
            (m + M) * ddx + B * dx + m * L / 2 * ddth * cos(th) - m * L / 2 * dth ** 2 * sin(th) - F      # EQUATION 2
        ],
            ddth, ddx)

        ddth = sympy.simplify(sol.get(ddth))
        ddx = sympy.simplify(sol.get(ddx))

        ddth = parse_expr(str(ddth).\
            replace('cos(th)', '1').replace('dth**2', '0').replace('sin(2*th)', '2*th').replace('sin(th)', 'th'))
        ddx = parse_expr(str(ddx).\
            replace('cos(th)', '1').replace('dth**2', '0').replace('sin(2*th)', '2*th').replace('sin(th)', 'th'))

        a21 = float(ddth.coeff(th))
        a22 = float(ddth.coeff(dth))
        a24 = float(ddth.coeff(dx))
        a41 = float(ddx.coeff(th))
        a42 = float(ddx.coeff(dth))
        a44 = float(ddx.coeff(dx))

        b2 = float(ddth.coeff(F))
        b4 = float(ddx.coeff(F))

        # STATE-SPACE PATTERN
        # x1 = dx
        # x2 = ddx
        # x3 = dth
        # x4 = ddth

        # [[dth], [ddth], [dx], [ddx]] = A*[[th], [dth], [x], [dx]] + B*u
        # y = C*[[th], [dth], [x], [dx]] + D*u
        #
        #  [dth ]              [th ]
        #  [ddth]              [dth]
        #  [dx  ]   =  [A]  *  [x  ]    +   [B] * U
        #  [ddx ]              [dx ]

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

        self.K, S, E = ctrl.lqr(ss, Q, R)
        self.K = abs(self.K).tolist()[0]
