import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

v = 5.     #5m/s
L = 2.3    #2.3m

u = 2. * np.pi / 180    #degrees in radians


def system_dynamics(_t, z):
    return [z[1],
            -z[0]*z[1] + u]

t_final = 2
#the initial conditions is:
#z(0) = [x(0), y(0) and theta(0)]
z_initial = [1, 1, 0.]
solution = solve_ivp(system_dynamics,
                     [0, t_final],
                     z_initial,
                     t_eval=np.linspace(0, t_final, 100))    #[0, ......1000]

times = solution.t
x_trajectory = solution.y[0]
y_trajectory = solution.y[1]
theta_trajectory = solution.y[2]

plt.plot(times, x_trajectory)

plt.xlabel('time (s)')
plt.ylabel('x position (m)')
plt.grid()
plt.show()


