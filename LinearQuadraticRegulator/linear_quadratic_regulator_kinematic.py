import math
import numpy as np
from enum import Enum
import matplotlib.pyplot as plt

# Vehicle config
wheelbase = 2.33  # wheel base: front to rear axle [m]
wheeldist = 1.85  # wheel dist: left to right wheel [m]
v_w = 2.33  # vehicle width [m]
r_b = 0.80  # rear to back [m]
r_f = 3.15  # rear to front [m]
t_r = 0.40  # tire radius [m]
t_w = 0.30  # tire width [m]

c_f = 155494.663  # [N/rad]
c_r = 155494.663  # [N/rad]
m_f = 570  # [kg]
m_r = 570  # [kg]
l_f = 1.165  # [m]
l_r = 1.165  # [m]
Iz = 1436.24  # [N*m]

# Controller Config
ts = 0.10  # [s]
max_iteration = 100
eps = 1e-4

matrix_q = [1.0, 1.0, 1.0]
matrix_r = [1.0, 1.0]

state_size = 3

max_acceleration = 5.0  # [m/ss]
max_steer_angle = np.deg2rad(40)  # [rad]
max_speed = 30 / 3.6  # [m/s]


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


class VehicleState:
    def __init__(self, x, y, yaw, v, wheelbase, dt):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.wheelbase = wheelbase
        self.dt = dt

    def UpdateVehicleState(self, delta_f, a):
        """
        update states of vehicle
        :param delta: steering angle [rad]
        :param a: acceleration [m/ss]
        """

        delta, a = self.RegulateInput(delta_f, a)  # 限制车辆转角与加速度

        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt

        self.yaw += self.v / self.wheelbase * math.tan(delta_f) * self.dt
        self.yaw = pi_2_pi(self.yaw)

        self.v += a * self.dt
        self.v = self.RegulateOutput(self.v)  # 限制车辆速度

    def state_space_equation(self, ref_delta, ref_yaw):

        A = np.matrix([
            [1.0, 0.0, -self.v * self.dt * math.sin(ref_yaw)],
            [0.0, 1.0, self.v * self.dt * math.cos(ref_yaw)],
            [0.0, 0.0, 1.0]])

        B = np.matrix([
            [self.dt * math.cos(ref_yaw), 0],
            [self.dt * math.sin(ref_yaw), 0],
            [self.dt * math.tan(ref_delta) / self.wheelbase, self.v * self.dt / (self.wheelbase * math.cos(ref_delta) * math.cos(ref_delta))]])

        return A, B

    @staticmethod
    def RegulateInput(delta, a):
        """
        regulate delta to : - max_steer_angle ~ max_steer_angle
        regulate a to : - max_acceleration ~ max_acceleration
        :param delta: steering angle [rad]
        :param a: acceleration [m/ss]
        :return: regulated delta and acceleration
        """

        if delta < -1.0 * max_steer_angle:
            delta = -1.0 * max_steer_angle

        if delta > 1.0 * max_steer_angle:
            delta = 1.0 * max_steer_angle

        if a < -1.0 * max_acceleration:
            a = -1.0 * max_acceleration

        if a > 1.0 * max_acceleration:
            a = 1.0 * max_acceleration

        return delta, a

    @staticmethod
    def RegulateOutput(v):
        """
        regulate v to : -max_speed ~ max_speed
        :param v: calculated speed [m/s]
        :return: regulated speed
        """

        max_speed_ = max_speed

        if v < -1.0 * max_speed_:
            v = -1.0 * max_speed_

        if v > 1.0 * max_speed_:
            v = 1.0 * max_speed_

        return v


class TrajectoryAnalyzer:
    def __init__(self):
        self.yaw_, self.k_ = [], []
        self.x_ = np.linspace(0, 100, 1000)  # x
        self.y_ = 2 * np.sin(self.x_ / 3.0) + 2.5 * np.cos(self.x_ / 2.0)  # y
        for i in range(len(self.x_)):
            if i == 0:
                dx = self.x_[i + 1] - self.x_[i]
                dy = self.y_[i + 1] - self.y_[i]
                ddx = self.x_[i + 2] + self.x_[i] - 2 * self.x_[i + 1]
                ddy = self.y_[i + 2] + self.y_[i] - 2 * self.y_[i + 1]
            elif i == (len(self.x_) - 1):
                dx = self.x_[i] - self.x_[i - 1]
                dy = self.y_[i] - self.y_[i - 1]
                ddx = self.x_[i - 2] + self.x_[i] - 2 * self.x_[i - 1]
                ddy = self.y_[i - 2] + self.y_[i] - 2 * self.y_[i - 1]
            else:
                dx = self.x_[i + 1] - self.x_[i]
                dy = self.y_[i + 1] - self.y_[i]
                ddx = self.x_[i - 1] + self.x_[i + 1] - 2 * self.x_[i]
                ddy = self.y_[i - 1] + self.y_[i + 1] - 2 * self.y_[i]
            self.yaw_.append(math.atan2(dy, dx))  # yaw
            self.k_.append((ddy * dx - ddx * dy) /
                           ((dx ** 2 + dy ** 2) ** (3 / 2)))  # k

    def ToTrajectoryFrame(self, vehicle_state):
        """
        errors to trajectory frame
        """

        x_cg = vehicle_state.x
        y_cg = vehicle_state.y

        # calc nearest point in ref path 索引值与距离值
        dx = [x_cg - self.x_[ix] for ix in range(len(self.x_))]
        dy = [y_cg - self.y_[iy] for iy in range(len(self.x_))]
        idx = int(np.argmin(np.hypot(dx, dy)))

        yaw_ref = self.yaw_[idx]
        k_ref = self.k_[idx]

        return yaw_ref, k_ref, idx


class LatController:
    """
    Lateral Controller using LQR 横向控制器LQR
    """

    def ComputeControlCommand(self, vehicle_state, ref_trajectory):
        """
        calc lateral control command.
        :param vehicle_state: vehicle state
        :param ref_trajectory: reference trajectory (analyzer)
        """

        yaw_ref, k_ref, target_idx = \
            ref_trajectory.ToTrajectoryFrame(vehicle_state)
        delta_ref = math.atan2(wheelbase * k_ref, 1)

        matrix_ad_, matrix_bd_ = vehicle_state.state_space_equation(
            delta_ref, yaw_ref)

        matrix_state_ = np.zeros((state_size, 1))
        matrix_r_ = np.diag(matrix_r)
        matrix_q_ = np.diag(matrix_q)

        matrix_k_ = self.SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_,
                                         matrix_r_, eps, max_iteration)

        matrix_state_[0][0] = vehicle_state.x - ref_trajectory.x_[target_idx]
        matrix_state_[1][0] = vehicle_state.y - ref_trajectory.y_[target_idx]
        matrix_state_[2][0] = vehicle_state.yaw - ref_trajectory.yaw_[target_idx]

        steer_angle_feedback = -(matrix_k_ @ matrix_state_)[1][0]  # 反馈

        steer_angle_feedforward = self.ComputeFeedForward(k_ref)  # 前馈

        steer_angle = steer_angle_feedback + delta_ref

        return steer_angle

    @staticmethod
    def ComputeFeedForward(ref_curvature):
        """
        calc feedforward control term to decrease the steady error. 消除稳定误差
        :param ref_curvature: curvature of the target point in ref trajectory
        :return: feedforward term
        """

        wheelbase_ = l_f + l_r

        steer_angle_feedforward = wheelbase_ * ref_curvature

        return steer_angle_feedforward

    @staticmethod
    def SolveLQRProblem(A, B, Q, R, tolerance, max_num_iteration):
        """
        iteratively calculating feedback matrix K
        :param A: matrix_a_
        :param B: matrix_b_
        :param Q: matrix_q_
        :param R: matrix_r_
        :param tolerance: lqr_eps
        :param max_num_iteration: max_iteration
        :return: feedback matrix K
        """

        # 0 -> rows 1 -> columns
        assert np.size(A, 0) == np.size(A, 1) and \
            np.size(B, 0) == np.size(A, 0) and \
            np.size(Q, 0) == np.size(Q, 1) and \
            np.size(Q, 0) == np.size(A, 1) and \
            np.size(R, 0) == np.size(R, 1) and \
            np.size(R, 0) == np.size(B, 1), \
            "LQR solver: one or more matrices have incompatible dimensions."

        M = np.zeros((np.size(Q, 0), np.size(R, 1)))

        AT = A.T
        BT = B.T
        MT = M.T

        P = Q
        num_iteration = 0
        diff = math.inf

        while num_iteration < max_num_iteration and diff > tolerance:
            num_iteration += 1
            P_next = AT @ P @ A - (AT @ P @ B + M) @ \
                np.linalg.pinv(R + BT @ P @ B) @ (BT @ P @ A + MT) + Q

            # check the difference between P and P_next
            diff = (abs(P_next - P)).max()
            P = P_next

        if num_iteration >= max_num_iteration:
            print("LQR solver cannot converge to a solution",
                  "last consecutive result diff is: ", diff)

        K = np.linalg.inv(BT @ P @ B + R) @ (BT @ P @ A + MT)

        return K


class LonController:
    """
    Longitudinal Controller using PID. 纵向控制器PID
    """

    @staticmethod
    def ComputeControlCommand(target_speed, vehicle_state, dist):
        """
        calc acceleration command using PID.
        :param target_speed: target speed [m/s]
        :param vehicle_state: vehicle state
        :param dist: distance to goal [m]
        :return: control command (acceleration) [m/ss]
        """

        direct = 1.0

        a = 0.3 * (target_speed - direct * vehicle_state.v)

        if dist < 10.0:
            if vehicle_state.v > 2.0:
                a = -3.0
            elif vehicle_state.v < -2.0:
                a = -1.0

        return a


def pi_2_pi(angle):
    """
    regulate theta to -pi ~ pi.
    :param angle: input angle
    :return: regulated angle
    """

    M_PI = math.pi

    if angle > M_PI:
        return angle - 2.0 * M_PI

    if angle < -M_PI:
        return angle + 2.0 * M_PI

    return angle


def main():

    x0, y0, yaw0, t = 0, -3, 0, 0

    ref_trajectory = TrajectoryAnalyzer()
    goal_x = ref_trajectory.x_[-1]
    goal_y = ref_trajectory.y_[-1]

    vehicle_state = VehicleState(x=x0, y=y0, yaw=yaw0, v=0, wheelbase=wheelbase, dt=ts)
    path_x, path_y, path_yaw = [], [], []
    maxTime = 100.0

    lat_controller = LatController()
    lon_controller = LonController()


    while t < maxTime:

        dist = math.hypot(vehicle_state.x - goal_x, vehicle_state.y - goal_y)
        target_speed = 25.0 / 3.6

        delta_opt = lat_controller.ComputeControlCommand(
            vehicle_state, ref_trajectory)

        a_opt = lon_controller.ComputeControlCommand(
            target_speed, vehicle_state, dist)

        vehicle_state.UpdateVehicleState(delta_opt, a_opt)

        t += ts

        if dist <= 0.5:
            break

        path_x.append(vehicle_state.x)
        path_y.append(vehicle_state.y)
        path_yaw.append(vehicle_state.yaw)

        x0 = path_x[-1]
        y0 = path_y[-1]
        yaw0 = path_yaw[-1]

        plt.cla()
        plt.plot(ref_trajectory.x_, ref_trajectory.y_,
                 color='gray', linewidth=2.0)
        plt.plot(path_x, path_y, linewidth=2.0, color='darkviolet')
        update_vehicle(x0, y0, yaw0, 0)
        plt.title("LQR (Kinematic): v=" +
                  str(vehicle_state.v * 3.6)[:4] + "km/h")
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event:
                                     [exit(0) if event.key == 'escape' else None])
        plt.pause(0.001)

    plt.show()


if __name__ == '__main__':
    main()
