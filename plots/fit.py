from sys import argv
import numpy as np
from scipy.optimize import curve_fit
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
            [lambda x: p(x, a1, b1, c1, d1), lambda x: p(x, a2_calculated, b2, c2, d2_calculated)])

if __name__ == '__main__' :
    with open(argv[1]) as data_file:
        data = np.array(list(map(float, l.split()) for l in data_file.read().split('\n') if l))
    readings = data[:,0]
    ranges = data[:,1]

    v0_guess = 600.

    cutoff_i = 0
    while readings[cutoff_i] < v0_guess: cutoff_i += 1

    #initial_params_1 = [994.82, 83.322, -0.388, -46.38]
    initial_params_1 = [3000.] + list(curve_fit(lambda x, b, c, d: p(x, 3000, b, c, d), readings[:cutoff_i], ranges[:cutoff_i], p0 = (0, -.28, -200), maxfev = 10**6)[0])
    print initial_params_1
    a2_calculated = a2(initial_params_1[0], initial_params_1[1], initial_params_1[2], initial_params_1[3], 500., 3., v0_guess)
    d2_calculated = d2(initial_params_1[0], initial_params_1[1], initial_params_1[2], initial_params_1[3], 500., 3., v0_guess)
    initial_params_2 = curve_fit(lambda x, b, c: p(x, a2_calculated, b, c, d2_calculated), readings[cutoff_i-1:], ranges[cutoff_i-1:], p0 = (500., 3.))[0]
    print initial_params_2

    plt.plot(readings, ranges, '.')
    plt.plot(readings, f(readings, *(list(initial_params_1)+list(initial_params_2)+[v0_guess])))
    plt.plot(readings, f(readings, 994.82, 83.322, -0.3888, -46.38366, -4925.18, 81.82, 760.66))
    plt.show()
    print 'Fit:'
    fit = curve_fit(f, readings, ranges, p0 = list(initial_params_1) + list(initial_params_2) + [v0_guess], maxfev = 10**6)
    fit_params = fit[0]
    print fit_params
    print 'Error:'
    print fit[1]

    print 'Final params:'
    print '{',
    for i in range(4):
        print str(fit_params[i]) + ',',
    print str(a2(*fit_params)) + ',',
    print str(fit_params[4]) + ',',
    print str(fit_params[5]) + ',',
    print str(d2(*fit_params)) + ',',
    print str(fit_params[-1]),
    print '}'

    plt.plot(readings, ranges, '.')
    plt.plot(readings, f(readings, *fit_params))
    plt.show()
