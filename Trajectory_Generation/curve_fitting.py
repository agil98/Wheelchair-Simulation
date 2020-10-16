#!/usr/bin/env python

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline, interp1d, splprep, splev


data_x = [0.8, 1.4, 1.8, 2.2, 2.8, 3.4, 3.8, 4, 4.2, 4.4, 4.8]
# data_y = [0.75, 1.25, 1, 1.6, 1.45, 1.9, 1.75, 2.05 ]

data_y = [1, 3, 1, 2, 1.45, 2.5, 2, 2.01, 2, 2.01, 2.05]

new_x = np.linspace(0.8, 4.8, 100)

# Linear
f = interp1d(data_x, data_y)
linear_y = f(new_x)

# Cubic Spline
cs = CubicSpline(data_x, data_y)
spline_y = cs(new_x)

# BSpline
tck, u = splprep([data_x, data_y], s=0.5)
new_points = splev(np.linspace(0, 1, 1000), tck)

plt.figure()
plt.scatter(data_x, data_y, label="Generated Points")
plt.plot(new_x, spline_y, label="Cubic Spline")
plt.plot(new_x, linear_y, label="Linear")
plt.plot(new_points[0], new_points[1], label="BSpline")
plt.xlabel("Time (s)")
plt.ylabel("Angular Veclocity (rad/s)")
plt.grid()
plt.legend()

plt.show()
