import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

v = 5.     #5m/s
L = 2.3    #2.3m

u = -2. * np.pi / 180    #degrees in radians


def system_dynamics(t, z):
    theta = z[2]
    return [v * np.cos(theta),
            v * np.sin(theta),
            v * np.tan(u) / L]


t_final = 2
#the initial conditions is:
#z(0) = [x(0), y(0) and theta(0)]
z_initial = [0., 0.3, 0.0873]
solution = solve_ivp(system_dynamics,
                     [0, t_final],
                     z_initial,
                     t_eval=np.linspace(0, t_final, 1000))    #[0, ......1000]

times = solution.t
x_trajectory = solution.y[0]
y_trajectory = solution.y[1]
theta_trajectory = solution.y[2]

plt.plot(times, y_trajectory)
plt.xlabel('time (s)')
plt.ylabel('y position (m)')
plt.grid()
plt.show()


