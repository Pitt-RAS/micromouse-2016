#!/usr/bin/env python2

'''
Script for fitting rangefinder data to a smooth curve

Fits data to a continuous and differentiable piecewise curve with two regions,
where each part is of the form a*(x-b)**c + d.

Output is (a1, b1, c1, d1, a2, b2, c2, d2, v0), where the 1's correspond to
parameters for x < v0 and the 2's correspond to parameters for x > v0.

Usage: python fit.py <data file name>
'''

__author__ = 'Aaron Miller'
__license__ = 'GPLv2'

import numpy as np
import scipy.optimize
import matplotlib.pyplot as plt

def a2(a1, b1, c1, d1, b2, c2, v0):
    return a1 * c1 / c2 * (v0 - b1)**(c1 - 1) / (v0 - b2)**(c2 - 1)

def d2(a1, b1, c1, d1, b2, c2, v0):
    a2_calculated = a2(a1, b1, c1, d1, b2, c2, v0)
    return a1 * (v0 - b1)**c1 + d1 - a2_calculated * (v0 - b2)**c2

def p(x, a, b, c, d):
    return a*(x-b)**c+d

def f(x, a1, b1, c1, d1, b2, c2, v0):
    a2_calculated = a2(a1, b1, c1, d1, b2, c2, v0)
    d2_calculated = d2(a1, b1, c1, d1, b2, c2, v0)
    return np.piecewise(
            x,
            [x < v0, x >= v0],
            [
                lambda x: p(x, a1, b1, c1, d1),
                lambda x: p(x, a2_calculated, b2, c2, d2_calculated)
            ])

def cost_function(params, readings, ranges):
    return sum((f(readings, *params) - ranges)**2) / float(len(readings))

def best_params_for_v0(v0_guess, readings, ranges):
    print 'v0:', v0_guess
    inflection_index = min(i
                           for i in range(len(readings))
                           if readings[i] >= v0_guess)

    (a1, b1, c1, d1), _ = scipy.optimize.curve_fit(
            p,
            readings[:inflection_index],
            ranges[:inflection_index],
            p0=(1, 0, -2, 0),
            maxfev=10**6)

    b2_guess, c2_guess = scipy.optimize.minimize(
            lambda (b2, c2): sum(f(readings[inflection_index:],
                          a1,
                          b1,
                          c1,
                          d1,
                          b2,
                          c2, v0_guess)
                          - ranges[inflection_index:]
                        )**2,
            (v0_guess - 1, 2),
            bounds=((None, v0_guess - 1), (1, None))
        ).x

    print 'cost:', cost_function((a1, b1, c1, d1, b2_guess, c2_guess, v0_guess),
                                 readings,
                                 ranges)
    return (a1, b1, c1, d1, b2_guess, c2_guess, v0_guess)

def best_params(readings, ranges):
    v0_guess = scipy.optimize.minimize_scalar(
            lambda v0: cost_function(best_params_for_v0(v0, readings, ranges),
                                     readings,
                                     ranges),
            bounds=(min(readings) + 1, max(readings) - 1),
            method='bounded'
        ).x

    optimize_result = scipy.optimize.minimize(
            lambda params: cost_function(params, readings, ranges),
            best_params_for_v0(v0_guess, readings, ranges),
            constraints=(
                    {
                        'type': 'ineq',
                        'fun': (lambda params: params[6] - params[1] - 1)
                    },
                    {
                        'type': 'ineq',
                        'fun': (lambda params: params[6] - params[4] - 1)
                    }
                ),
            options={ 'maxiter': 10**4 }
        )

    print optimize_result
    return optimize_result.x

def main(filename):
    with open(filename) as data_file:
        data = np.array([map(float, l.split())
                         for l in data_file.read().split('\n')
                         if l])

    data = np.array(sorted(data, key=lambda p: p[0]))
    readings = data[:,0]
    ranges = data[:,1]

    params = best_params(readings, ranges)

    print 'Final params:'
    print params[0],
    print params[1],
    print params[2],
    print params[3],
    print a2(*params),
    print params[4],
    print params[5],
    print d2(*params),
    print params[6]

    plt.plot(readings, f(readings, *params))
    plt.plot(readings, ranges)
    plt.show()

if __name__ == '__main__' :
    import sys
    main(sys.argv[1])
