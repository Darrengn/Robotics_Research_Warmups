import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


def u(t):
    return np.array([1, 1])

def s_dot(t, s, tstep):
    x, y, theta = s
    v, w = u(t)
    return np.array([tstep*v*np.cos(theta) + x, tstep*v*np.sin(theta) + y, w*tstep + theta])

s0 = np.array([0, 0, 0])
tstep = 0.5
numsteps = 20
sol = np.zeros((3,numsteps+1))
t = np.arange(numsteps+1) * tstep

sol[:,0] = s0
cur_s = s0

for i in range(numsteps):
    s_new = s_dot(i * tstep, cur_s, tstep)
    sol[:,i+1] = s_new
    cur_s = s_new

print(sol)

plt.plot(t, sol[0], label='x(t)')
plt.show()
plt.plot(t, sol[1], label='y(t)')
plt.show()