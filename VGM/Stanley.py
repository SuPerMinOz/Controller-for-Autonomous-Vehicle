import math
import numpy as np
import matplotlib.pyplot as plt
import cubic_spline as cs

# PID config
Kp = 1.0

# System config
k = 1
dt = 0.1
dstop = 0.5
protect_velocity = 0.2 / 3.6
preview_index = 5
MAX_STEER = 40 * math.pi / 180
MAX_CURVE = 0.2

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
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, a, delta):

        delta = self.limit_input(delta)

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.yaw = pi_2_pi(self.yaw)
        self.v += a * dt

    @staticmethod
    def limit_input(delta):
        if delta > MAX_STEER:
            return MAX_STEER

        if delta < -MAX_STEER:
            return -MAX_STEER

        return delta


class Trajectory:
    def __init__(self, cx, cy, cyaw, ck):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.last_target_idx = 0

    def calc_error(self, vehicle):

        fx = vehicle.x + WB * math.cos(vehicle.yaw)
        fy = vehicle.y + WB * math.sin(vehicle.yaw)

        dx = [fx - x for x in self.cx]
        dy = [fy - y for y in self.cy]

        target_index = int(np.argmin(np.hypot(dx, dy)))
        # no more backward motion
        target_index = max(self.last_target_idx, target_index)

        # preview control at large curvature road
        ref_kappa = self.ck[target_index]
        if (abs(ref_kappa) > MAX_CURVE):
            target_index += preview_index
            if (target_index >= len(self.cx)):
                target_index = len(self.cx) - 1
        self.last_target_idx = max(self.last_target_idx, target_index)

        ref_kappa = self.ck[target_index]

        lateral_error = - dy[target_index] * math.cos(vehicle.yaw) + \
            dx[target_index] * math.sin(vehicle.yaw)

        ref_theta = self.cyaw[target_index]
        cur_theta = vehicle.yaw
        theta_error = pi_2_pi(ref_theta - cur_theta)

        return theta_error, lateral_error, target_index, ref_kappa


def stanley_control(vehicle, ref_path):

    theta_error, lateral_error, target_index, ref_kappa = ref_path.calc_error(
        vehicle)
    delta = theta_error + \
        math.atan2(k * lateral_error, max(vehicle.v,
                   protect_velocity)) - 0.5 * ref_kappa
    delta = pi_2_pi(delta)

    return delta, target_index


def pid_control(target_v, v, dist):

    a = 0.3 * (target_v - v)

    if dist < 10.0:
        if v > 3.0:
            a = -2.5
        elif v < -2.0:
            a = -1.0

    return a


def main():
    # generate path
    ax = np.arange(0, 50, 0.5)
    ay = [math.sin(ix / 5.0) * ix / 2.0 for ix in ax]
    target_speed = 25.0 / 3.6

    # ax = np.linspace(0, 100, 1000)
    # ay = 2 * np.sin(ax / 3.0) + 2.5 * np.cos(ax / 2.0)
    # target_speed = 10.0 / 3.6

    cx, cy, cyaw, ck, _ = cs.calc_spline_course(ax, ay, ds=dt)
    ref_path = Trajectory(cx, cy, cyaw, ck)

    t = 0.0
    maxTime = 100.0

    # init position
    x0, y0, yaw0 = cx[0], cy[0], cyaw[0]
    xrec, yrec, yawrec = [], [], []

    vehicle = Vehicle(x=x0, y=y0, yaw=yaw0, v=0.0)

    while t < maxTime:
        t += dt

        di, target_index = stanley_control(vehicle, ref_path)

        dist = math.hypot(vehicle.x - cx[-1], vehicle.y - cy[-1])
        ai = pid_control(target_speed, vehicle.v, dist)

        vehicle.update(ai, di)

        if dist <= dstop:
            break

        xrec.append(vehicle.x)
        yrec.append(vehicle.y)
        yawrec.append(vehicle.yaw)

        plt.cla()
        plt.plot(cx, cy, color='gray', linewidth=2.0)
        plt.plot(xrec, yrec, linewidth=2.0, color='darkviolet')
        plt.plot(cx[target_index], cy[target_index], '.r')
        update_vehicle(vehicle.x, vehicle.y, vehicle.yaw, 0)
        plt.axis("equal")
        plt.title("Stanley Controller: v=" + str(vehicle.v * 3.6)[:4] + "km/h")
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event:
                                     [exit(0) if event.key == 'escape' else None])
        plt.pause(0.001)

    plt.show()


if __name__ == '__main__':
    main()
