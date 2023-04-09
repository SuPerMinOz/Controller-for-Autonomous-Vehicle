import math
import numpy as np
import matplotlib.pyplot as plt
import reeds_shepp as rs


# PID config
Kp = 0.3  # proportional gain

# system config
kf = 0.1  # look forward gain
dt = 0.1  # T step
MAX_STEER = 45 * np.pi / 180
MAX_CURVE = 0.3
MAX_ACCELERATION = 5.0

# vehicle config
RF = 3.3  # [m] distance from rear to vehicle front end of vehicle
RB = 0.8  # [m] distance from rear to vehicle back end of vehicle
W = 2.4  # [m] width of vehicle
WD = 0.7 * W  # [m] distance between left-right wheels
WB = 2.5  # [m] Wheel base
TR = 0.44  # [m] Tyre radius
TW = 0.7  # [m] Tyre width


class Arrow:
    def __init__(self, x, y, theta, L, c):
        angle = np.deg2rad(30)
        d = 0.4 * L
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
    car = np.array([[-RB, -RB, RF, RF, -RB],
                    [W / 2, -W / 2, -W / 2, W / 2, W / 2]])

    wheel = np.array([[-TR, -TR, TR, TR, -TR],
                      [TW / 4, -TW / 4, -TW / 4, TW / 4, TW / 4]])

    rlWheel = wheel.copy()
    rrWheel = wheel.copy()
    frWheel = wheel.copy()
    flWheel = wheel.copy()

    Rot1 = np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])

    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    frWheel = np.dot(Rot2, frWheel)
    flWheel = np.dot(Rot2, flWheel)

    frWheel += np.array([[WB], [-WD / 2]])
    flWheel += np.array([[WB], [WD / 2]])
    rrWheel[1, :] -= WD / 2
    rlWheel[1, :] += WD / 2

    frWheel = np.dot(Rot1, frWheel)
    flWheel = np.dot(Rot1, flWheel)

    rrWheel = np.dot(Rot1, rrWheel)
    rlWheel = np.dot(Rot1, rlWheel)
    car = np.dot(Rot1, car)

    frWheel += np.array([[x], [y]])
    flWheel += np.array([[x], [y]])
    rrWheel += np.array([[x], [y]])
    rlWheel += np.array([[x], [y]])
    car += np.array([[x], [y]])

    plt.plot(car[0, :], car[1, :], color)
    plt.plot(frWheel[0, :], frWheel[1, :], color)
    plt.plot(rrWheel[0, :], rrWheel[1, :], color)
    plt.plot(flWheel[0, :], flWheel[1, :], color)
    plt.plot(rlWheel[0, :], rlWheel[1, :], color)
    Arrow(x, y, yaw, WB * 0.6, color)


def pi_2_pi(angle):
    if angle > math.pi:
        return angle - 2.0 * math.pi
    if angle < -math.pi:
        return angle + 2.0 * math.pi
    return angle


class Vehicle:
    def __init__(self, x, y, yaw, v, direct):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct

    def update(self, a, delta, direct):

        delta = self.limit_input(delta)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        self.yaw += self.v / WB * math.tan(delta) * dt
        self.yaw = pi_2_pi(self.yaw)

        self.direct = direct
        self.v += self.direct * a * dt

    @staticmethod
    def limit_input(delta):
        if delta > 1.2 * MAX_STEER:
            return 1.2 * MAX_STEER

        if delta < -1.2 * MAX_STEER:
            return -1.2 * MAX_STEER

        return delta


class VehicleCache:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.direct = []

    def add(self, t, vehicle):
        self.x.append(vehicle.x)
        self.y.append(vehicle.y)
        self.yaw.append(vehicle.yaw)
        self.v.append(vehicle.v)
        self.t.append(t)
        self.direct.append(vehicle.direct)


class Trajectory:
    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.idx_end = len(self.cx)
        self.idx_start = None

    def calc_target_index(self, vehicle, Ld=2.5):

        if self.idx_start is None:
            self.calc_nearest_ind(vehicle)

        Lf = kf * vehicle.v + Ld

        for ind in range(self.idx_start, self.idx_end):
            if self.calc_distance(vehicle, ind) >= Lf:
                self.idx_start = ind
                return ind, Lf

        self.idx_start = self.idx_end - 1

        return self.idx_end - 1, Lf

    def calc_nearest_ind(self, vehicle):
        dx = [vehicle.x - x for x in self.cx]
        dy = [vehicle.y - y for y in self.cy]
        ind = np.argmin(np.hypot(dx, dy))
        self.idx_start = ind

    def calc_distance(self, vehicle, ind):
        return math.hypot(vehicle.x - self.cx[ind], vehicle.y - self.cy[ind])


def pure_pursuit(vehicle, ref_path, idx_start, Ld):
    # target point and pursuit distance
    ind, Lf = ref_path.calc_target_index(vehicle, Ld)
    ind = max(ind, idx_start)

    path_x = ref_path.cx[ind]
    path_y = ref_path.cy[ind]

    alpha = math.atan2(path_y - vehicle.y, path_x - vehicle.x) - vehicle.yaw
    alpha = pi_2_pi(alpha)

    delta = math.atan2(2.0 * WB * math.sin(alpha), Lf)

    return delta, ind


def pid_control(target_v, v, dist, direct):

    a = 0.3 * (target_v - direct * v)

    if dist < 10.0:
        if v > 3.0:
            a = -2.5
        elif v < -2.0:
            a = -1.0

    return a


def generate_path(s):
    """
    divide paths into some sections, in each section, the direction is the same.
    :param s: target position and yaw
    :return: sections
    """

    max_c = math.tan(MAX_CURVE) / WB  # max curvature

    path_x, path_y, yaw, direct = [], [], [], []
    x_rec, y_rec, yaw_rec, direct_rec = [], [], [], []
    direct_flag = 1.0

    for i in range(len(s) - 1):
        s_x, s_y, s_yaw = s[i][0], s[i][1], np.deg2rad(s[i][2])
        g_x, g_y, g_yaw = s[i + 1][0], s[i + 1][1], np.deg2rad(s[i + 1][2])
        path_i = rs.calc_optimal_path(s_x, s_y, s_yaw, g_x, g_y, g_yaw, max_c)

        ix = path_i.x
        iy = path_i.y
        iyaw = path_i.yaw
        idirect = path_i.directions

        for j in range(len(ix)):
            if idirect[j] == direct_flag:
                x_rec.append(ix[j])
                y_rec.append(iy[j])
                yaw_rec.append(iyaw[j])
                direct_rec.append(idirect[j])
            else:
                if len(x_rec) == 0 or direct_rec[0] != direct_flag:
                    direct_flag = idirect[j]
                    continue

                path_x.append(x_rec)
                path_y.append(y_rec)
                yaw.append(yaw_rec)
                direct.append(direct_rec)
                x_rec, y_rec, yaw_rec, direct_rec = \
                    [x_rec[-1]], [y_rec[-1]], [yaw_rec[-1]], [-direct_rec[-1]]

    path_x.append(x_rec)
    path_y.append(y_rec)
    yaw.append(yaw_rec)
    direct.append(direct_rec)

    x_all, y_all = [], []

    for ix, iy in zip(path_x, path_y):
        x_all += ix
        y_all += iy

    return path_x, path_y, yaw, direct, x_all, y_all


def main():
    # generate path
    states = [(0, 0, 0), (20, 15, 0), (35, 20, 90), (40, 0, 180),
              (20, 0, 120), (5, -10, 180), (15, 5, 30)]

    # states = [(-3, 3, 120), (10, -7, 30), (10, 13, 30), (20, 5, -25),
    #           (35, 10, 180), (30, -10, 160), (5, -12, 90)]

    x, y, yaw, direct, path_x, path_y = generate_path(states)

    maxTime = 100.0
    x0, y0, yaw0, direct0 = x[0][0], y[0][0], yaw[0][0], direct[0][0]
    x_rec, y_rec = [], []

    for cx, cy, _, cdirect in zip(x, y, yaw, direct):
        t = 0.0
        vehicle = Vehicle(x=x0, y=y0, yaw=yaw0, v=0.0, direct=direct0)
        vehiclecache = VehicleCache()
        vehiclecache.add(t, vehicle)
        ref_trajectory = Trajectory(cx, cy)
        target_ind, _ = ref_trajectory.calc_target_index(vehicle)

        while t <= maxTime:
            if cdirect[0] > 0:
                target_speed = 30.0 / 3.6
                Ld = 4.0
                dist_stop = 1.5
                distance_to_center = -1.1
            else:
                target_speed = 20.0 / 3.6
                Ld = 2.5
                dist_stop = 0.2
                distance_to_center = 0.2

            xt = vehicle.x + distance_to_center * math.cos(vehicle.yaw)
            yt = vehicle.y + distance_to_center * math.sin(vehicle.yaw)
            dist = math.hypot(xt - cx[-1], yt - cy[-1])

            if dist < dist_stop:
                break

            acceleration = pid_control(
                target_speed, vehicle.v, dist, cdirect[0])
            delta, target_ind = pure_pursuit(
                vehicle, ref_trajectory, target_ind, Ld)

            t += dt

            vehicle.update(acceleration, delta, cdirect[0])
            vehiclecache.add(t, vehicle)
            x_rec.append(vehicle.x)
            y_rec.append(vehicle.y)

            x0 = vehiclecache.x[-1]
            y0 = vehiclecache.y[-1]
            yaw0 = vehiclecache.yaw[-1]
            direct0 = vehiclecache.direct[-1]

            # animation
            plt.cla()
            plt.plot(vehicle.x, vehicle.y, marker='.', color='k')
            plt.plot(path_x, path_y, color='gray', linewidth=2)
            plt.plot(x_rec, y_rec, color='darkviolet', linewidth=2)
            plt.plot(cx[target_ind], cy[target_ind], ".r")
            update_vehicle(vehicle.x, vehicle.y, vehicle.yaw, 0)

            for m in range(len(states)):
                Arrow(states[m][0], states[m][1],
                      np.deg2rad(states[m][2]), 2, 'blue')

            plt.axis("equal")
            plt.title("PurePursuit: v=" + str(vehicle.v * 3.6)[:4] + "km/h")
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event:
                                         [exit(0) if event.key == 'escape' else None])
            plt.pause(0.001)

    plt.show()


def main_another():
    t = 0.0
    maxTime = 100

    cx = np.linspace(0, 100, 1000)
    cy = 2 * np.sin(cx / 3.0) + 2.5 * np.cos(cx / 2.0)
    ref_trajectory = Trajectory(cx, cy)

    x0, y0, yaw0, direct0 = 0, -3, 0, 1
    x_rec, y_rec = [], []
    vehicle = Vehicle(x=x0, y=y0, yaw=yaw0, v=0.0, direct=direct0)
    vehiclecache = VehicleCache()
    vehiclecache.add(t, vehicle)
    target_ind, _ = ref_trajectory.calc_target_index(vehicle)

    while t <= maxTime:
        target_speed = 10.0 / 3.6
        Ld = 2.0
        dist_stop = 1.5
        distance_to_center = -1.1

        xt = vehicle.x + distance_to_center * math.cos(vehicle.yaw)
        yt = vehicle.y + distance_to_center * math.sin(vehicle.yaw)
        dist = math.hypot(xt - cx[-1], yt - cy[-1])
        if dist < dist_stop:
            break

        acceleration = pid_control(
            target_speed, vehicle.v, dist, direct0)
        delta, target_ind = pure_pursuit(
            vehicle, ref_trajectory, target_ind, Ld)

        t += dt

        vehicle.update(acceleration, delta, direct0)
        vehiclecache.add(t, vehicle)
        x_rec.append(vehicle.x)
        y_rec.append(vehicle.y)

        x0 = vehiclecache.x[-1]
        y0 = vehiclecache.y[-1]
        yaw0 = vehiclecache.yaw[-1]
        direct0 = vehiclecache.direct[-1]

        # animation
        plt.cla()
        plt.plot(vehicle.x, vehicle.y, marker='.', color='k')
        plt.plot(cx, cy, color='gray', linewidth=2)
        plt.plot(x_rec, y_rec, color='darkviolet', linewidth=2)
        plt.plot(cx[target_ind], cy[target_ind], ".r")
        update_vehicle(vehicle.x, vehicle.y, vehicle.yaw, 0)


        plt.axis("equal")
        plt.title("PurePursuit: v=" + str(vehicle.v * 3.6)[:4] + "km/h")
        plt.gcf().canvas.mpl_connect('key_release_event',
                                        lambda event:
                                        [exit(0) if event.key == 'escape' else None])
        plt.pause(0.001)

    plt.show()


if __name__ == '__main__':
    main()
    # main_another()
