import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def u(t):
    return np.array([1, 1])

def s_dot(t, s):
    x, y, theta = s
    v, w = u(t)
    return np.array([v*np.cos(theta), v*np.sin(theta), w])

s0 = np.array([0, 0, 0])
tspan = (0,2)

sol = solve_ivp(s_dot, tspan, s0, method='RK45', t_eval=np.linspace(0, 2, 50))
t = sol.t
plt.plot(t, sol.y[0], label='x(t)')
plt.show()
plt.plot(t, sol.y[1], label='y(t)')
plt.show()