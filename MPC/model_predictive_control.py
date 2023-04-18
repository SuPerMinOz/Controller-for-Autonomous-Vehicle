import math
import cvxpy
import numpy as np
import matplotlib.pyplot as plt

import cubic_spline as cs

# System config
NX = 4  # state vector: z = [x, y, v, phi]
NU = 2  # input vector: u = [acceleration, steer]
T = 6  # finite time horizon length

# MPC config
Q = np.diag([1.0, 1.0, 1.0, 10.0])  # penalty for states
Qf = np.diag([1.0, 1.0, 1.0, 1.0])  # penalty for end state
R = np.diag([0.01, 0.1])  # penalty for inputs
Rd = np.diag([0.01, 0.1])  # penalty for change of inputs

dist_stop = 1.5  # stop permitted when dist to goal < dist_stop
speed_stop = 0.5 / 3.6  # stop permitted when speed < speed_stop
time_max = 500.0  # max simulation time
iter_max = 5  # max iteration
target_speed = 10.0 / 3.6  # target speed
N_IND = 10  # search index number
dt = 0.2  # time step
d_dist = 1.0  # dist step
du_res = 0.1  # threshold for stopping iteration

# vehicle config
r_f = 3.3  # [m] distance from rear to vehicle front end of vehicle
r_b = 0.8  # [m] distance from rear to vehicle back end of vehicle
v_w = 2.4  # vehicle width [m]
wheeldist = 0.7 * v_w  # wheel dist: left to right wheel [m]
wheelbase = 2.5  # wheel base: front to rear axle [m]
t_r = 0.44  # [m] Tyre radius
t_w = 0.5  # [m] Tyre width

steer_max = np.deg2rad(45.0)  # max steering angle [rad]
steer_change_max = np.deg2rad(30.0)  # maximum steering speed [rad/s]
speed_max = 55.0 / 3.6  # maximum speed [m/s]
speed_min = -20.0 / 3.6  # minimum speed [m/s]
acceleration_max = 1.0  # maximum acceleration [m/s2]


class Arrow:
    def __init__(self, x, y, theta, L, c):
        angle = np.deg2rad(30)
        d = 0.3 * L
        w = 2

        x_start = x
        y_start = y
        x_end = x + L * np.cos(theta)
        y_end = y + L * np.sin(theta)

        theta_hat_L = theta + math.pi - angle
        theta_hat_R = theta + math.pi + angle

        x_hat_start = x_end
        x_hat_end_L = x_hat_start + d * np.cos(theta_hat_L)
        x_hat_end_R = x_hat_start + d * np.cos(theta_hat_R)

        y_hat_start = y_end
        y_hat_end_L = y_hat_start + d * np.sin(theta_hat_L)
        y_hat_end_R = y_hat_start + d * np.sin(theta_hat_R)

        plt.plot([x_start, x_end], [y_start, y_end], color=c, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_L],
                 [y_hat_start, y_hat_end_L], color=c, linewidth=w)
        plt.plot([x_hat_start, x_hat_end_R],
                 [y_hat_start, y_hat_end_R], color=c, linewidth=w)


def update_vehicle(x, y, yaw, steer, color='black'):
    vehicle = np.array([[-r_b, -r_b, r_f, r_f, -r_b],
                        [v_w / 2, -v_w / 2, -v_w / 2, v_w / 2, v_w / 2]])

    wheel = np.array([[-t_r, -t_r, t_r, t_r, -t_r],
                      [t_w / 2, -t_w / 2, -t_w / 2, t_w / 2, t_w / 2]])

    rlWheel = wheel.copy()
    rrWheel = wheel.copy()
    frWheel = wheel.copy()
    flWheel = wheel.copy()

    Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)],
                     [np.sin(yaw), np.cos(yaw)]])

    Rot2 = np.array([[np.cos(steer), np.sin(steer)],
                     [-np.sin(steer), np.cos(steer)]])

    frWheel = np.dot(Rot2, frWheel)
    flWheel = np.dot(Rot2, flWheel)

    frWheel += np.array([[wheelbase], [-wheeldist / 2]])
    flWheel += np.array([[wheelbase], [wheeldist / 2]])
    rrWheel[1, :] -= wheeldist / 2
    rlWheel[1, :] += wheeldist / 2

    frWheel = np.dot(Rot1, frWheel)
    flWheel = np.dot(Rot1, flWheel)

    rrWheel = np.dot(Rot1, rrWheel)
    rlWheel = np.dot(Rot1, rlWheel)
    vehicle = np.dot(Rot1, vehicle)

    frWheel += np.array([[x], [y]])
    flWheel += np.array([[x], [y]])
    rrWheel += np.array([[x], [y]])
    rlWheel += np.array([[x], [y]])
    vehicle += np.array([[x], [y]])

    plt.plot(vehicle[0, :], vehicle[1, :], color)
    plt.plot(frWheel[0, :], frWheel[1, :], color)
    plt.plot(rrWheel[0, :], rrWheel[1, :], color)
    plt.plot(flWheel[0, :], flWheel[1, :], color)
    plt.plot(rlWheel[0, :], rlWheel[1, :], color)
    Arrow(x, y, yaw, 0.8 * wheelbase, color)
    plt.axis("equal")


class Vehicle:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, direct=1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct

    def update(self, a, delta, direct):

        delta = self.limit_input_delta(delta)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        self.yaw += self.v / wheelbase * math.tan(delta) * dt
        # self.yaw = pi_2_pi(self.yaw)

        self.direct = direct
        self.v += self.direct * a * dt
        self.v = self.limit_speed(self.v)

    @staticmethod
    def limit_input_delta(delta):
        if delta >= steer_max:
            return steer_max

        if delta <= -steer_max:
            return -steer_max

        return delta

    @staticmethod
    def limit_speed(v):
        if v >= speed_max:
            return speed_max

        if v <= speed_min:
            return speed_min

        return v


class TrajectoryAnalyzer:
    def __init__(self, cx, cy, cyaw, ck):
        self.cx = cx
        self.cy = cy
        self.cyaw = smooth_yaw(cyaw)
        self.ck = ck
        self.length = len(cx)
        self.idx_start = 0

    def nearest_index(self, vehicle):
        """
        calc index of the nearest node in N steps
        :param vehicle: current information
        :return: nearest index, lateral distance to ref point
        """

        dx = [
            vehicle.x - x for x in self.cx[self.idx_start: (self.idx_start + N_IND)]]
        dy = [
            vehicle.y - y for y in self.cy[self.idx_start: (self.idx_start + N_IND)]]
        dist = np.hypot(dx, dy)

        ind_in_N = int(np.argmin(dist))
        ind = self.idx_start + ind_in_N
        self.idx_start = ind

        rear_axle_vec_rot_90 = np.array([[-math.sin(vehicle.yaw)],
                                         [math.cos(vehicle.yaw)]])

        vec_target_2_rear = np.array([[dx[ind_in_N]],
                                      [dy[ind_in_N]]])

        lateral_error = np.dot(vec_target_2_rear.T, rear_axle_vec_rot_90)

        return ind, lateral_error


def calc_ref_trajectory_in_T_step(vehicle, ref_path, speed_profile):
    """
    calc referent trajectory in T steps: [x, y, v, yaw]
    using the current velocity, calc the T points along the reference path
    :param vehicle: current information
    :param ref_path: reference path: [x, y, yaw]
    :param speed_profile: speed profile (designed speed strategy)
    :return: reference trajectory
    """

    z_ref = np.zeros((NX, T + 1))
    length = ref_path.length

    ind, _ = ref_path.nearest_index(vehicle)

    z_ref[0, 0] = ref_path.cx[ind]
    z_ref[1, 0] = ref_path.cy[ind]
    z_ref[2, 0] = speed_profile[ind]
    z_ref[3, 0] = ref_path.cyaw[ind]

    dist_move = 0.0

    for i in range(1, T + 1):
        dist_move += abs(vehicle.v) * dt
        ind_move = int(round(dist_move / d_dist))
        index = min(ind + ind_move, length - 1)

        z_ref[0, i] = ref_path.cx[index]
        z_ref[1, i] = ref_path.cy[index]
        z_ref[2, i] = speed_profile[index]
        z_ref[3, i] = ref_path.cyaw[index]

    return z_ref, ind


def linear_mpc_control(z_ref, z0, a_opt, delta_opt):
    """
    linear mpc controller
    :param z_ref: reference trajectory in T steps
    :param z0: initial state vector
    :param a_opt: acceleration of T steps of last time
    :param delta_opt: delta of T steps of last time
    :return: acceleration and delta strategy based on current information
    """

    if a_opt is None or delta_opt is None:
        a_opt = [0.0] * T
        delta_opt = [0.0] * T

    x, y, yaw, v = None, None, None, None

    for k in range(iter_max):
        z_bar = predict_states_in_T_step(z0, a_opt, delta_opt, z_ref)
        a_rec, delta_rec = a_opt[:], delta_opt[:]
        a_opt, delta_opt, x, y, yaw, v = solve_linear_mpc(
            z_ref, z_bar, z0, delta_opt)

        du_a_max = max([abs(ia - iao) for ia, iao in zip(a_opt, a_rec)])
        du_d_max = max([abs(ide - ido)
                       for ide, ido in zip(delta_opt, delta_rec)])

        if max(du_a_max, du_d_max) < du_res:
            break

    return a_opt, delta_opt, x, y, yaw, v


def predict_states_in_T_step(z0, a, delta, z_ref):
    """
    given the current state, using the acceleration and delta strategy of last time,
    predict the states of vehicle in T steps.
    :param z0: initial state
    :param a: acceleration strategy of last time
    :param delta: delta strategy of last time
    :param z_ref: reference trajectory
    :return: predict states in T steps (z_bar, used for calc linear motion model)
    """

    z_bar = z_ref * 0.0

    for i in range(NX):
        z_bar[i, 0] = z0[i]

    vehicle = Vehicle(x=z0[0], y=z0[1], v=z0[2], yaw=z0[3])

    for ai, di, i in zip(a, delta, range(1, T + 1)):
        vehicle.update(ai, di, 1.0)
        z_bar[0, i] = vehicle.x
        z_bar[1, i] = vehicle.y
        z_bar[2, i] = vehicle.v
        z_bar[3, i] = vehicle.yaw

    return z_bar


def calc_linear_discrete_model(v, phi, delta):
    """
    calc linear and discrete time dynamic model.
    :param v: speed: v_bar
    :param phi: angle of vehicle: phi_bar
    :param delta: steering angle: delta_bar
    :return: A, B, C
    """

    A = np.array([[1.0, 0.0, dt * math.cos(phi), - dt * v * math.sin(phi)],
                  [0.0, 1.0, dt * math.sin(phi), dt * v * math.cos(phi)],
                  [0.0, 0.0, 1.0, 0.0],
                  [0.0, 0.0, dt * math.tan(delta) / wheelbase, 1.0]])

    B = np.array([[0.0, 0.0],
                  [0.0, 0.0],
                  [dt, 0.0],
                  [0.0, dt * v / (wheelbase * math.cos(delta) ** 2)]])

    C = np.array([dt * v * math.sin(phi) * phi,
                  -dt * v * math.cos(phi) * phi,
                  0.0,
                  -dt * v * delta / (wheelbase * math.cos(delta) ** 2)])

    return A, B, C


def solve_linear_mpc(z_ref, z_bar, z0, d_bar):
    """
    solve the quadratic optimization problem using cvxpy, solver: OSQP
    :param z_ref: reference trajectory (desired trajectory: [x, y, v, yaw])
    :param z_bar: predicted states in T steps
    :param z0: initial state
    :param d_bar: delta_bar
    :return: optimal acceleration and steering strategy
    """

    z = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constrains = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)
        cost += cvxpy.quad_form(z_ref[:, t] - z[:, t], Q)

        A, B, C = calc_linear_discrete_model(
            z_bar[2, t], z_bar[3, t], d_bar[t])

        constrains += [z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C]

        if t < T - 1:
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constrains += [cvxpy.abs(u[1, t + 1] - u[1, t])
                           <= steer_change_max * dt]

    cost += cvxpy.quad_form(z_ref[:, T] - z[:, T], Qf)

    constrains += [z[:, 0] == z0]
    constrains += [z[2, :] <= speed_max]
    constrains += [z[2, :] >= speed_min]
    constrains += [cvxpy.abs(u[0, :]) <= acceleration_max]
    constrains += [cvxpy.abs(u[1, :]) <= steer_max]

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
    prob.solve(solver=cvxpy.OSQP)

    a, delta, x, y, yaw, v = None, None, None, None, None, None

    if prob.status == cvxpy.OPTIMAL or \
            prob.status == cvxpy.OPTIMAL_INACCURATE:
        x = z.value[0, :]
        y = z.value[1, :]
        v = z.value[2, :]
        yaw = z.value[3, :]
        a = u.value[0, :]
        delta = u.value[1, :]
    else:
        print("Cannot solve linear mpc!")

    return a, delta, x, y, yaw, v


def calc_speed_profile(cx, cy, cyaw, target_speed):
    """
    design appropriate speed strategy
    :param cx: x of reference path [m]
    :param cy: y of reference path [m]
    :param cyaw: yaw of reference path [m]
    :param target_speed: target speed [m/s]
    :return: speed profile
    """

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def pi_2_pi(angle):
    if angle > math.pi:
        return angle - 2.0 * math.pi

    if angle < -math.pi:
        return angle + 2.0 * math.pi

    return angle


def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def main():
    # ax = [0.0, 15.0, 30.0, 50.0, 60.0]
    # ay = [0.0, 40.0, 15.0, 30.0, 0.0]

    ax = [60.0, 50.0, 30.0, 15.0, 0.0]
    ay = [0.0, 30.0, 15.0, 40.0, 0.0]

    cx, cy, cyaw, ck, s = cs.calc_spline_course(
        ax, ay, ds=d_dist)
    
    speed_profile = calc_speed_profile(cx, cy, cyaw, target_speed)

    ref_path = TrajectoryAnalyzer(cx, cy, cyaw, ck)
    vehicle = Vehicle(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

    time = 0.0
    x = [vehicle.x]
    y = [vehicle.y]
    yaw = [vehicle.yaw]
    v = [vehicle.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]

    delta_opt, a_opt = None, None
    a_exc, delta_exc = 0.0, 0.0

    while time < time_max:
        z_ref, target_ind = \
            calc_ref_trajectory_in_T_step(vehicle, ref_path, speed_profile)

        z0 = [vehicle.x, vehicle.y, vehicle.v, vehicle.yaw]

        a_opt, delta_opt, x_opt, y_opt, yaw_opt, v_opt = \
            linear_mpc_control(z_ref, z0, a_opt, delta_opt)

        if delta_opt is not None:
            delta_exc, a_exc = delta_opt[0], a_opt[0]

        vehicle.update(a_exc, delta_exc, 1.0)
        time += dt

        x.append(vehicle.x)
        y.append(vehicle.y)
        yaw.append(vehicle.yaw)
        v.append(vehicle.v)
        t.append(time)
        d.append(delta_exc)
        a.append(a_exc)

        dist = math.hypot(vehicle.x - cx[-1], vehicle.y - cy[-1])

        if dist < dist_stop and \
                abs(vehicle.v) < speed_stop:
            break

        dy = (vehicle.yaw - yaw[-2]) / (vehicle.v * dt)
        steer = pi_2_pi(-math.atan(wheelbase * dy))

        plt.cla()
        update_vehicle(vehicle.x, vehicle.y, vehicle.yaw, steer)
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event:
                                     [exit(0) if event.key == 'escape' else None])

        if x_opt is not None:
            plt.plot(x_opt, y_opt, color='darkviolet', marker='*')

        plt.plot(cx, cy, color='gray')
        plt.plot(x, y, '-b')
        plt.plot(cx[target_ind], cy[target_ind])
        plt.axis("equal")
        plt.title("Linear MPC, " + "v = " +
                  str(round(vehicle.v * 3.6, 2)) + "km/h")
        plt.pause(0.001)

    plt.show()


if __name__ == '__main__':
    main()
