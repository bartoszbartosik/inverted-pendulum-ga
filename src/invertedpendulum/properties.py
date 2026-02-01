from dataclasses import dataclass, field


@dataclass
class _Pendulum:
    length: float = 0.8         # [m]
    mass: float = 0.3           # [kg]
    friction: float = 0.05      # [kg/s]
    max_ang: float = 9.0e+9     # [deg]
    max_vel: float = 12         # [rad/s]
    max_acc: float = 9.0e+9     # [deg/s²]

@dataclass
class _Cart:
    mass: float = 0.6           # [kg]
    friction: float = 0.1       # [1/s]
    max_pos: float = 5          # [m]
    max_vel: float = 4          # [m/s]
    max_acc: float = 9.0e+9     # [m/s²]
    max_force: float = 9.0e+9   #


@dataclass
class Properties:

    G: float = -9.81

    pendulum: _Pendulum = field(default_factory=_Pendulum)
    cart: _Cart = field(default_factory=_Cart)
