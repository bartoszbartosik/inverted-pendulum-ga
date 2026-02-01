import numpy as np

import sympy
from sympy import symbols
from sympy import sin, cos
from sympy.parsing.sympy_parser import parse_expr

import control as ctrl

from .properties import Properties


class IPEngine:
    """
    Equation 1
    (I + m * L ^ 2 / 4) * ddth + m * g * L / 2 * sin(th) + b * dth = -m * L / 2 * ddx * cos(th)
    (I + m*L**2/4)*FI*s**2 + m*g*L/2*th + b*FI*s = -m*L/2*X*s**2
    Equation 2
    (m + M) * ddx + B * dx + m * L / 2 * ddth * cos(th) - m * L / 2 * dth ^ 2 * sin(th) = F
    (m + M)*X*s**2 + B*X*s + m*L/2*FI*s**2 = F
    """

    def __init__(self):

        # ------------------------- #
        #   R E A D   C O N F I G   #
        # ------------------------- #

        self.properties = Properties()

        # Pendulum's parameters
        self.L = self.properties.pendulum.length
        self.m = self.properties.pendulum.mass
        self.b = self.properties.pendulum.friction
        self.I = 1 / 3 * self.m * self.L ** 2

        # Cart's parameters
        self.M = self.properties.cart.mass
        self.B = self.properties.cart.friction

        # Gravitational acceleration
        self.G = self.properties.G


        # ----------------------------------- #
        #   C O M P U T E   D Y N A M I C S   #
        # ----------------------------------- #

        # Input values
        self.__th, self.__dth, self.__dx, self.__F = symbols('th, dth, dx, F')

        # Output values
        self.__ddth, self.__ddx = symbols('ddth ddx')

        self.ddth_symb, self.ddx_symb = self.solve_for_derivatives()
        self.ddth_func, self.ddx_func = self.__get_derivative_functions()


    def solve_for_derivatives(self, symbolic=False):

        if symbolic:
            L, m, b, B, M, g, I = symbols('L m b B M g I')
        else:
            L, m, b, B, M, g, I = self.L, self.m, self.b, self.B, self.M, self.G, self.I

        # Solve ODEs
        sol = sympy.solve([
            # Equation 1
            (I + m * L ** 2 / 4) * self.__ddth + m * g * L / 2 * sin(self.__th) + b * self.__dth + m * L / 2 * self.__ddx * cos(self.__th),
            # Equation 2
            (m + M) * self.__ddx + B * self.__dx + m * L / 2 * self.__ddth * cos(self.__th) - m * L / 2 * self.__dth ** 2 * sin(self.__th) - self.__F
        ],self.__ddth, self.__ddx)

        ddth = sympy.simplify(sol.get(self.__ddth))
        ddx = sympy.simplify(sol.get(self.__ddx))

        return ddth, ddx


    def __get_derivative_functions(self):
        ddth_func = sympy.lambdify((self.__th, self.__dth, self.__dx, self.__F), self.ddth_symb, modules='numpy')
        ddx_func = sympy.lambdify((self.__th, self.__dth, self.__dx, self.__F), self.ddx_symb, modules='numpy')
        return ddth_func, ddx_func


    def __linearize_expression(self, expr):
        return parse_expr(str(expr).replace('cos(th)', '1').replace('dth**2', '0').replace('sin(2*th)', '2*th').replace('sin(th)', 'th').replace('th**2', '0'))


    def compute_ss_matrices(self, symbolic=False):
        """
        x1 = dx
        x2 = ddx
        x3 = dth
        x4 = ddth

        [[dth], [ddth], [dx], [ddx]] = A*[[th], [dth], [x], [dx]] + B*u
        y = C*[[th], [dth], [x], [dx]] + D*u

        [ dth  ]              [ th  ]
        [ ddth ]              [ dth ]
        [ dx   ] =  [ A ] *   [ x   ]   +   [ B ] * [ U ]
        [ ddx  ]              [ dx  ]
        """

        ddth, ddx = self.solve_for_derivatives(symbolic=symbolic)

        # Linearize
        ddth = self.__linearize_expression(ddth)
        ddx = self.__linearize_expression(ddx)

        if symbolic:
            a21 = sympy.simplify(ddth.expand().coeff(self.__th))
            a22 = sympy.simplify(ddth.expand().coeff(self.__dth))
            a24 = sympy.simplify(ddth.expand().coeff(self.__dx))
            a41 = sympy.simplify(ddx.expand().coeff(self.__th))
            a42 = sympy.simplify(ddx.expand().coeff(self.__dth))
            a44 = sympy.simplify(ddx.expand().coeff(self.__dx))

            b2 = sympy.simplify(self.ddth_symb.expand().coeff(self.__F))
            b4 = sympy.simplify(self.ddx_symb.expand().coeff(self.__F))
        else:

            a21 = float(ddth.coeff(self.__th))
            a22 = float(ddth.coeff(self.__dth))
            a24 = float(ddth.coeff(self.__dx))
            a41 = float(ddx.coeff(self.__th))
            a42 = float(ddx.coeff(self.__dth))
            a44 = float(ddx.coeff(self.__dx))

            b2 = float(ddth.coeff(self.__F))
            b4 = float(ddx.coeff(self.__F))


        A = np.matrix([
          # [th,    dth,    x,  dx]
            [0,     1,      0,  0],     # dth
            [a21,   a22,    0,  a24],   # ddth
            [0,     0,      0,  1],     # dx
            [a41,   a42,    0,  a44]    # ddx
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

        return A, B, C, D


    def compute_lqr(self, A, B, C, D):
        ss = ctrl.ss(
            A, B, C, D,
            states=['th', 'dth', 'x', 'dx'],
            inputs=['F'],
            outputs=['th', 'x']
        )

        Q = np.diag([1, 100, 22500, 5000])
        R = 1
        K, S, E = ctrl.lqr(ss, Q, R)
        return abs(K)[0]


if __name__ == '__main__':
    x = IPEngine()
    k = x.compute_ss_matrices()
    j = x.compute_ss_matrices(symbolic=True)
    pass
