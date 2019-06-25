import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.optimize import minimize_scalar

# Global constants
m = 0.1302 # mass
r = 0.0745 / 2 # distance from center of robot to wheel
I = 0.00015 # moment of inertia
mu = 1 # coefficient of friction
g = 9.8 # gravitational acceleration
i = I / m # moment of inertia per unit mass

# Constants that depend on velocity
def constants(v, turn_angle):
    tau_factor = v * r / i
    beta = mu * g * i / r / v**2
    accel_duration = np.pi / 2 / tau_factor
    accel_angle = beta * (1 - np.cos(tau_factor*accel_duration))
    if accel_angle > turn_angle / 2:
        accel_angle = turn_angle / 2
        accel_duration = np.arccos(1 - accel_angle/beta) / tau_factor
    max_omega = beta * tau_factor * np.sin(tau_factor*accel_duration)
    turn_duration = 2*accel_duration + (turn_angle - 2*accel_angle) / max_omega
    return {
            'tau_factor': tau_factor,
            'beta': beta,
            'accel_duration': accel_duration,
            'accel_angle': accel_angle,
            'max_omega': max_omega,
            'turn_duration': turn_duration
    }

# Theta as a function of time (shape of entire function depends on velocity)
def theta(t, v, turn_angle):
    c = constants(v, turn_angle)
    tau_factor = c['tau_factor']
    beta = c['beta']
    accel_duration = c['accel_duration']
    accel_angle = c['accel_angle']
    max_omega = c['max_omega']
    turn_duration = c['turn_duration']

    if not isinstance(t, np.ndarray):
        passed_scalar = True
        t = np.array([t])
    else:
        passed_scalar = False

    # The boolean expressions in these MUST use bitwise and, because the boolean
    # and operator doesn't work on numpy arrays
    result = np.piecewise(
        t,
        [
            t < 0,
            (0 <= t) & (t <= accel_duration),
            (accel_duration < t) & (t <= turn_duration - accel_duration),
            (turn_duration - accel_duration < t) & (t <= turn_duration),
            turn_duration < t
        ],
        [
            lambda t: 0,
            lambda t: beta * (1 - np.cos(tau_factor*t)),
            lambda t: accel_angle + (t - accel_duration) * max_omega,
            lambda t: turn_angle - theta(turn_duration - t, v, turn_angle),
            lambda t: turn_angle
        ])

    if passed_scalar:
        return result[0]
    else:
        return result

def trajectory(theta_func, v, turn_angle, t=None):
    if t is None:
        t = np.linspace(0, constants(v, turn_angle)['turn_duration'], 1000)
    x = odeint(lambda _, t: v * np.sin(theta_func(t)), 0, t)
    y = odeint(lambda _, t: v * np.cos(theta_func(t)), 0, t)
    return np.hstack((x, y))

def turn_size(v, turn_angle):
    traj = trajectory(lambda t: theta(t, v, turn_angle), v, turn_angle)
    endpoint = traj[-1,:]
    midpoint = traj[traj.shape[0]/2, :]
    min_radius = endpoint[0] / (1 - np.cos(turn_angle))
    max_radius = np.hypot(midpoint[0] - min_radius, midpoint[1])
    return min_radius
turn_size = np.vectorize(turn_size)
turn_size.excluded.add(1)

if __name__ == '__main__':
    angles = [
            np.pi / 4,
            np.pi / 2,
            3*np.pi/4,
            np.pi
        ]
    radii = [
            0.09 / np.tan(np.pi / 8),
            0.09,
            0.18 * np.sqrt(2) * np.tan(np.pi / 8),
            0.09
        ]

    for j in range(4):
        turn_angle = angles[j]
        target_radius = radii[j]
        v = np.linspace(0.5, 1.5)
        error_function = lambda v: (turn_size(v, turn_angle) - target_radius)**2
        result = minimize_scalar(error_function, bounds=(0, 10), method='bounded')
        print result
        v = result.x

        traj = trajectory(lambda t: theta(t, v, turn_angle), v, turn_angle)
        plt.axis('equal')
        plt.plot(*np.hsplit(traj, 2))
        r = turn_size(v, turn_angle)
        plt.plot([0, r, r * (1 - np.cos(turn_angle))],
                 [0, 0, r * np.sin(turn_angle)])
        plt.show()
