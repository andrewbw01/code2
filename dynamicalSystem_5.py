import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


class Car:

    def __init__(self, length=2.3, velocity=5, x=0, y=0, pose=0):
        self.__length = length
        self.__velocity = velocity
        self.__x = x
        self.__y = y
        self.__pose = pose      #pose is the angle

    def move(self, steering_angle, dt):
        #dt is the period of time for which angle is applied
        #simulate the motion of trajectory of car
        #z_initial = [self.x, self.y, self,pose]

        def bicycle_model(t, z):
            theta = z[2]
            return [self.__velocity * np.cos(theta),
                    self.__velocity * np.sin(theta),
                    self.__velocity * np.tan(steering_angle) / self.__length]

        z_initial = [self.__x, self.__y, self.__pose]
        solution = solve_ivp(bicycle_model,
                             [0, dt],
                             z_initial)
        self.__x = solution.y[0][-1]
        self.__y = solution.y[1][-1]
        self.__pose = solution.y[2][-1]

    def x(self):
        return self.__x

    def y(self):
        return self.__y

    def pose(self):
        return self.__pose

    def length(self):
        return self.__length


class PIDControler:

    #kp; proportional gain
    #kd: derivative gain
    #ki: integral gain
    #Ts: sampling time

    def __init__(self, kp, ki, kd, ts):
        """
        constructor for PID controller
        :param kp:
        :param ki:
        :param kd:
        :param ts:
        """
        self.__kp = kp
        self.__kd = kd / ts
        self.__ki = ki * ts
        self.__ts = ts
        self.__previous_error = None
        self.__sum_errors = 0.

    def control(self, y, y_set_point=0):
        error = y_set_point - y
        control_action = self.__kp * error

        if self.__previous_error is not None:
            control_action += self.__kd * (error - self.__previous_error)

        control_action += self.__ki * self.__sum_errors

        self.__sum_errors += error
        self.__previous_error = error #so that next time we need the previos error
                                      # we know where to find it
        return control_action

t_sampling = 0.01
pid = PIDControler(kp=0.1, ki=0.02, kd=0.15, ts=t_sampling)     #ki can get rid of the offset
murphy = Car(x=0, y=0.3, pose=0.0873)

y_cache = np.array([murphy.y()])
x_cache = np.array([murphy.x()])                         #we inserted the correct (first) value of y
pose_cache = np.array([murphy.pose()])
                                                         #into the y cache
num_points = 2000
for k in range(num_points):
    control_action = pid.control(y=murphy.y())
    murphy.move(control_action + 0.2, t_sampling)           #changing the off set
    y_cache = np.append(y_cache, murphy.y())
    x_cache = np.append(x_cache, murphy.x())
    pose_cache = np.append(pose_cache, murphy.pose())

t_span = t_sampling * np.arange(num_points + 1)
plt.plot(x_cache, y_cache)
plt.xlabel('horizontal position, x (m)')
plt.ylabel('lateral position, y (m)')
plt.grid()
plt.show()










