import math
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum
import reeds_shepp as rs

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
max_iteration = 150
eps = 0.01

matrix_q = [1.0, 0.0, 1.0, 0.0]
matrix_r = [1.0]

state_size = 4

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


class Gear(Enum):
    GEAR_DRIVE = 1
    GEAR_REVERSE = 2


class VehicleState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, gear=Gear.GEAR_DRIVE):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.lateral_error = 0.0
        self.yaw_error = 0.0

        self.gear = gear
        self.steer = 0.0

    def UpdateVehicleState(self, delta, a, lateral_error, yaw_error, gear=Gear.GEAR_DRIVE):
        """
        update states of vehicle
        :param yaw_error: yaw error to ref trajectory
        :param lateral_error: lateral error to ref trajectory
        :param delta: steering angle [rad]
        :param a: acceleration [m/ss]
        :param gear: gear mode [GEAR_DRIVE/GEAR_REVERSE]
        """

        wheelbase_ = l_r + l_f
        delta, a = self.RegulateInput(delta, a)  # 限制车辆转角与加速度

        self.gear = gear
        self.steer = delta
        self.x += self.v * math.cos(self.yaw) * ts
        self.y += self.v * math.sin(self.yaw) * ts
        self.yaw += self.v / wheelbase_ * math.tan(delta) * ts
        self.lateral_error = lateral_error
        self.yaw_error = yaw_error

        if gear == Gear.GEAR_DRIVE:
            self.v += a * ts
        else:
            self.v += -1.0 * a * ts

        self.v = self.RegulateOutput(self.v)  # 限制车辆速度

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
    def __init__(self, x, y, yaw, k):
        self.x_ = x
        self.y_ = y
        self.yaw_ = yaw
        self.k_ = k

        self.idx_start = 0
        self.idx_end = len(x)

    def ToTrajectoryFrame(self, vehicle_state):
        """
        errors to trajectory frame
        yaw_error = yaw_vehicle - yaw_ref_path
        lateral_error = lateral distance of center of gravity (cg) in frenet frame
        :param vehicle_state: vehicle state (class VehicleState)
        :return: yaw_error, lateral_error, yaw_ref, k_ref
        """

        x_cg = vehicle_state.x
        y_cg = vehicle_state.y
        yaw = vehicle_state.yaw

        # calc nearest point in ref path 索引值与距离值
        dx = [x_cg - ix for ix in self.x_[self.idx_start: self.idx_end]]
        dy = [y_cg - iy for iy in self.y_[self.idx_start: self.idx_end]]

        ind_add = int(np.argmin(np.hypot(dx, dy)))
        dist = math.hypot(dx[ind_add], dy[ind_add])

        # calc lateral relative position of vehicle to ref path
        # dy * cos(yaw) - dx * sin(yaw)
        vec_axle_rot = np.array([[-math.sin(yaw)], [math.cos(yaw)]])
        vec_path_2_cg = np.array([[dx[ind_add]], [dy[ind_add]]])

        if np.dot(vec_axle_rot.T, vec_path_2_cg) > 0.0:
            lateral_error = 1.0 * dist  # vehicle on the right of ref path
        else:
            lateral_error = -1.0 * dist  # vehicle on the left of ref path
        # lateral_error = np.dot(vec_axle_rot.T, vec_path_2_cg)

        # calc yaw error: yaw_error = yaw_vehicle - yaw_ref
        self.idx_start += ind_add
        yaw_ref = self.yaw_[self.idx_start]
        yaw_error = pi_2_pi(yaw - yaw_ref)

        # calc ref curvature
        k_ref = self.k_[self.idx_start]

        return yaw_error, lateral_error, yaw_ref, k_ref


class LatController:
    """
    Lateral Controller using LQR 横向控制器LQR
    """

    def ComputeControlCommand(self, vehicle_state, ref_trajectory):
        """
        calc lateral control command.
        :param vehicle_state: vehicle state
        :param ref_trajectory: reference trajectory (analyzer)
        :return: steering angle (optimal u), yaw_error, lateral_error
        """

        ts_ = ts
        last_lateral_error = vehicle_state.lateral_error
        last_yaw_error = vehicle_state.yaw_error

        yaw_error, lateral_error, yaw_ref, k_ref = \
            ref_trajectory.ToTrajectoryFrame(vehicle_state)

        matrix_ad_, matrix_bd_ = self.UpdateMatrix(vehicle_state)

        matrix_state_ = np.zeros((state_size, 1))
        matrix_r_ = np.diag(matrix_r)
        matrix_q_ = np.diag(matrix_q)

        matrix_k_ = self.SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_,
                                         matrix_r_, eps, max_iteration)

        matrix_state_[0][0] = lateral_error
        matrix_state_[1][0] = (lateral_error - last_lateral_error) / ts_
        matrix_state_[2][0] = yaw_error
        matrix_state_[3][0] = (yaw_error - last_yaw_error) / ts_

        steer_angle_feedback = -(matrix_k_ @ matrix_state_)[0][0]  # 反馈

        steer_angle_feedforward = self.ComputeFeedForward(k_ref)  # 前馈

        steer_angle = steer_angle_feedback + steer_angle_feedforward

        return steer_angle, yaw_error, lateral_error

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

    @staticmethod
    def UpdateMatrix(vehicle_state):
        """
        calc A and b matrices of linearized, discrete system.
        :return: A, b
        """

        ts_ = ts
        wheelbase_ = l_f + l_r

        v = vehicle_state.v

        # time discrete A matrix
        matrix_ad_ = np.zeros((state_size, state_size))
        matrix_ad_[0][0] = 1.0
        matrix_ad_[0][1] = ts_
        matrix_ad_[1][2] = v
        matrix_ad_[2][2] = 1.0
        matrix_ad_[2][3] = ts_

        # b = [0.0, 0.0, 0.0, v / L].T
        # time discrete b matrix
        matrix_bd_ = np.zeros((state_size, 1))
        matrix_bd_[3][0] = v / wheelbase_

        return matrix_ad_, matrix_bd_


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

        if vehicle_state.gear == Gear.GEAR_DRIVE:
            direct = 1.0
        else:
            direct = -1.0

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


def generate_path(s):
    """
    design path using reeds-shepp path generator.
    divide paths into sections, in each section the direction is the same.
    :param s: objective positions and directions.
    :return: paths
    """
    wheelbase_ = l_f + l_r

    max_c = math.tan(0.5 * max_steer_angle) / wheelbase_
    path_x, path_y, yaw, direct, rc = [], [], [], [], []
    x_rec, y_rec, yaw_rec, direct_rec, rc_rec = [], [], [], [], []
    direct_flag = 1.0

    for i in range(len(s) - 1):
        s_x, s_y, s_yaw = s[i][0], s[i][1], np.deg2rad(s[i][2])
        g_x, g_y, g_yaw = s[i + 1][0], s[i + 1][1], np.deg2rad(s[i + 1][2])

        path_i = rs.calc_optimal_path(s_x, s_y, s_yaw,
                                      g_x, g_y, g_yaw, max_c)

        irc, rds = rs.calc_curvature(
            path_i.x, path_i.y, path_i.yaw, path_i.directions)

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
                rc_rec.append(irc[j])
            else:
                if len(x_rec) == 0 or direct_rec[0] != direct_flag:
                    direct_flag = idirect[j]
                    continue

                path_x.append(x_rec)
                path_y.append(y_rec)
                yaw.append(yaw_rec)
                direct.append(direct_rec)
                rc.append(rc_rec)
                x_rec, y_rec, yaw_rec, direct_rec, rc_rec = \
                    [x_rec[-1]], [y_rec[-1]], [yaw_rec[-1]
                                               ], [-direct_rec[-1]], [rc_rec[-1]]

    path_x.append(x_rec)
    path_y.append(y_rec)
    yaw.append(yaw_rec)
    direct.append(direct_rec)
    rc.append(rc_rec)

    x_all, y_all = [], []
    for ix, iy in zip(path_x, path_y):
        x_all += ix
        y_all += iy

    return path_x, path_y, yaw, direct, rc, x_all, y_all


def main():
    # generate path
    states = [(0, 0, 0), (20, 15, 0), (35, 20, 90), (40, 0, 180),
              (20, 0, 120), (5, -10, 180), (15, 5, 30)]

    # states = [(-3, 3, 120), (10, -7, 30), (10, 13, 30), (20, 5, -25),
    #           (35, 10, 180), (30, -10, 160), (5, -12, 90)]

    x_ref, y_ref, yaw_ref, direct, curv, x_all, y_all = generate_path(states)

    wheelbase_ = l_f + l_r

    maxTime = 100.0
    x0, y0, yaw0, direct0 = \
        x_ref[0][0], y_ref[0][0], yaw_ref[0][0], direct[0][0]

    x_rec, y_rec, yaw_rec, direct_rec = [], [], [], []

    lat_controller = LatController()
    lon_controller = LonController()

    for x, y, yaw, gear, k in zip(x_ref, y_ref, yaw_ref, direct, curv):
        t = 0.0

        if gear[0] == 1.0:
            direct = Gear.GEAR_DRIVE
        else:
            direct = Gear.GEAR_REVERSE

        ref_trajectory = TrajectoryAnalyzer(x, y, yaw, k)

        vehicle_state = VehicleState(x=x0, y=y0, yaw=yaw0, v=0.1, gear=direct)

        while t < maxTime:

            dist = math.hypot(vehicle_state.x - x[-1], vehicle_state.y - y[-1])

            if gear[0] > 0:
                target_speed = 25.0 / 3.6
            else:
                target_speed = 15.0 / 3.6

            delta_opt, yaw_error, lateral_error = \
                lat_controller.ComputeControlCommand(
                    vehicle_state, ref_trajectory)

            a_opt = lon_controller.ComputeControlCommand(
                target_speed, vehicle_state, dist)

            vehicle_state.UpdateVehicleState(
                delta_opt, a_opt, lateral_error, yaw_error, direct)

            t += ts

            if dist <= 0.5:
                break

            x_rec.append(vehicle_state.x)
            y_rec.append(vehicle_state.y)
            yaw_rec.append(vehicle_state.yaw)

            x0 = x_rec[-1]
            y0 = y_rec[-1]
            yaw0 = yaw_rec[-1]

            plt.cla()
            plt.plot(x_all, y_all, color='gray', linewidth=2.0)
            plt.plot(x_rec, y_rec, linewidth=2.0, color='darkviolet')
            update_vehicle(x0, y0, yaw0, -vehicle_state.steer)
            plt.axis("equal")
            plt.title("LQR (Kinematic): v=" +
                      str(vehicle_state.v * 3.6)[:4] + "km/h")
            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event:
                                         [exit(0) if event.key == 'escape' else None])
            plt.pause(0.001)

    plt.show()


if __name__ == '__main__':
    main()
