import os

import numpy as np
import pandas as pd

from legwheel.cbase.dls import dls_solve_3x3_c


def create_command_csv_phi(theta_command, beta_command, file_name, transform=True):  # 4*n, 4*n
    # Tramsform beta, theta to right, left motor angles
    theta_0 = np.array([-17, 17])*np.pi/180
    theta_beta = np.array([theta_command, beta_command]
                          ).reshape(2, -1)   # 2*(4*n)
    phi_r, phi_l = np.array(
        [[1, 1], [-1, 1]]) @ theta_beta + theta_0.reshape(2, 1)

    phi_r = phi_r.reshape(4, -1)    # 4*n
    phi_l = phi_l.reshape(4, -1)    # 4*n

    #### Tramsform (motor angle from 0 to initial pose) ####
    if transform:
        tramsform_r = []
        tramsform_l = []
        for i in range(4):
            tramsform_r.append(np.hstack(
                # finally 4*m
                (np.linspace(0, phi_r[i, 0], 5000), phi_r[i, 0]*np.ones(2000))))
            tramsform_l.append(
                np.hstack((np.linspace(0, phi_l[i, 0], 5000), phi_l[i, 0]*np.ones(2000))))
        phi_r = np.hstack((tramsform_r, phi_r))
        phi_l = np.hstack((tramsform_l, phi_l))

    # put into the format of motor command
    motor_command = np.empty((phi_r.shape[1], 2*phi_r.shape[0]))
    for i in range(4):
        motor_command[:, 2*i] = phi_r[i, :]
        motor_command[:, 2*i+1] = phi_l[i, :]

    # write motor commands into csv file #
    # add four column of -1
    motor_command = np.hstack(
        (motor_command, -1*np.ones((motor_command.shape[0], 4))))
    df = pd.DataFrame(motor_command)

    # 將 DataFrame 寫入 Excel 檔案
    df.to_csv(file_name + '.csv', index=False, header=False)


def create_command_csv(theta_command, beta_command, file_name, transform=True):  # 4*n, 4*n
    #### Tramsform (motor angle from 0 to initial pose) ####
    if transform:
        tramsform_theta = []
        tramsform_beta = []
        for i in range(4):
            tramsform_theta.append(np.hstack((np.linspace(np.deg2rad(
                # finally 4*m
                17), theta_command[i, 0], 5000), theta_command[i, 0]*np.ones(2000))))
            tramsform_beta.append(np.hstack(
                (np.linspace(0, beta_command[i, 0], 5000), beta_command[i, 0]*np.ones(2000))))
        theta_command = np.hstack((tramsform_theta, theta_command))
        beta_command = np.hstack((tramsform_beta, beta_command))

    # put into the format of motor command
    motor_command = np.empty(
        (theta_command.shape[1], 2*theta_command.shape[0]))
    for i in range(4):
        motor_command[:, 2*i] = theta_command[i, :]
        if i in [1, 2]:
            motor_command[:, 2*i+1] = beta_command[i, :]
        else:
            motor_command[:, 2*i+1] = -beta_command[i, :]

    # transfer motor command to be continuous, i.e. [pi-d, -pi+d] -> [pi-d, pi+d]
    # threshold = np.pi/2
    # last = motor_command[0,:]
    # for angle in motor_command[1:]:
    #     for i in range(8):
    #         while np.abs(angle[i]-last[i]) > threshold:
    #             angle[i] -= np.pi*np.sign(angle[i]-last[i])
    #     last = angle

    # write motor commands into csv file #
    # add four column of -1
    motor_command = np.hstack(
        (motor_command, -1*np.ones((motor_command.shape[0], 4))))
    df = pd.DataFrame(motor_command)

    # 將 DataFrame 寫入 Excel 檔案
    df.to_csv(file_name + '.csv', index=False, header=False)


# position, time: 0~1, acceleration time, initial velocity, final velocity
def parabolic_blends(p, t, tp=0.2, vi=0, vf=0):
    p = np.array(p).astype(float)
    t = np.array(t)
    n_points = p.shape[0]
    p0 = p[0]
    if vi is None:
        vi = (p[1]-p[0]) / (t[1]-t[0])
    else:
        t[0] += 0.5 * tp  # t0 = tp/2
        p[0] = p[0] + vi * tp/2
    if vf is None:
        vf = (p[-1]-p[-2]) / (t[-1]-t[-2])
    else:
        t[-1] -= 0.5 * tp
        p[-1] = p[-1] - vf * tp/2
    parabolic_arr = np.zeros((2*n_points-1, 3))
    v = np.hstack((vi, np.diff(p)/np.diff(t), vf))
    a = np.diff(v)/tp

    # a0 = ((p[1]-v[0]*tp[0]-p[0])/(t[1]-tp[0]) - v[0]) / ( 2*tp[0] + (tp[0])**2/(t[1]-tp[0]) )   # p0(t) = a t^2 + v0 t + p0
    # p[0] = np.polyval(np.array([a0, v[0], p[0]]), tp[0])    # p0(tp/2)
    # t[0] = tp[0]    # t0 = tp/2
    # v = np.hstack((vi, np.diff(p)/np.diff(t), vf))
    # a = np.diff(v)/tp
    # 1st segment, 0~tp, acceleration
    parabolic_arr[0] = np.array([0.5*a[0], v[0], p0])
    for i in range(n_points-1):
        if i == 0:
            # constant speed
            parabolic_arr[2*i+1] = v[i+1] * \
                np.array([0, 1, -t[i]]) + np.array([0, 0, p[i]])
        else:
            # constant speed
            parabolic_arr[2*i+1] = v[i+1] * \
                np.array([0, 1, -t[i]]) + np.array([0, 0, p[i]])
        tmp = t[i+1] - 0.5*tp  # acceleration start time
        parabolic_arr[2*i+2] = parabolic_arr[2*i+1] + 0.5 * \
            a[i+1]*np.array([1, -2*tmp, tmp**2])  # acceleration

    return parabolic_arr


def get_parabolic_point(p, parabolic_arr, t, tp=0.1):
    t = np.array(t)
    n_points = t.shape[0]
    t[0] += 0.5 * tp
    t[-1] -= 0.5 * tp

    segments = np.zeros(parabolic_arr.shape[0])
    segments[0] = t[0] + 0.5 * tp
    for i in range(n_points-1):
        segments[2*i+1] = t[i+1] - 0.5 * tp
        segments[2*i+2] = t[i+1] + 0.5 * tp

    if p < 0:
        return np.polyval(parabolic_arr[0], p/p)
    elif p >= 1.0:
        return np.polyval(parabolic_arr[-1], p/p)
    else:
        for idx, segment in enumerate(segments):
            if p < segment:
                return np.polyval(parabolic_arr[idx], p)

    print("ERROR IN get_parabolic_point")
    return 0


def numerical_jacobian(func, q, *args, diff=1e-5, **kwargs):
    """
    Calculates the numerical Jacobian matrix using forward finite difference.

    Args:
        func (callable): The function `f(q)` to differentiate. Can return a scalar or a numpy array.
        q (np.ndarray): The input variables at which to evaluate the Jacobian (e.g., joint angles).
        diff (float): The step size for numerical differentiation.
        *args, **kwargs: Additional arguments passed to `func`.

    Returns:
        np.ndarray: The Jacobian matrix evaluated at `q`.
    """
    q = np.array(q, dtype=float)
    f0 = np.array(func(q, *args, **kwargs))

    # Determine output dimensions
    m = f0.size
    n = q.size
    J = np.zeros((m, n))

    for i in range(n):
        q_plus = q.copy()
        q_plus[i] += diff
        f_plus = np.array(func(q_plus, *args, **kwargs))
        J[:, i] = (f_plus.flatten() - f0.flatten()) / diff

    # If the output is a scalar, reshape to 1D
    if m == 1:
        return J.flatten()
    return J


def pseudo_inverse_dls(J, damping_factor=1e-2):
    """
    Calculates the pseudo-inverse of a Jacobian matrix using Damped Least Squares (DLS)
    (also known as the Levenberg-Marquardt method). Useful near singularities.

    J* = J^T (J J^T + lambda^2 I)^-1

    Args:
        J (np.ndarray): The m x n Jacobian matrix.
        damping_factor (float): The damping factor (lambda) for avoiding singularity.

    Returns:
        np.ndarray: The n x m damped pseudo-inverse matrix.
    """
    J = np.array(J)
    if J.ndim == 1:
        J = J.reshape(1, -1)

    m, n = J.shape
    # Ensure I has the right dimension (m x m) for J @ J.T + I
    I = np.eye(m)
    J_star = J.T @ np.linalg.inv(J @ J.T + (damping_factor**2) * I)
    return J_star


def dls_solve(J, rhs, damping_factor=1e-2):
    """
    Solve q_dot = J* rhs using DLS with optional C backend.

    When LEGWHEEL_USE_CBASE=1 and J/rhs are 3x3/3x1, this dispatches to a
    compiled C kernel. Otherwise it falls back to NumPy DLS.
    """
    J = np.array(J, dtype=float)
    rhs = np.array(rhs, dtype=float).reshape(-1)

    use_c = os.getenv("LEGWHEEL_USE_CBASE", "0") == "1"
    if use_c and J.shape == (3, 3) and rhs.shape == (3,):
        try:
            return dls_solve_3x3_c(J, rhs, damping_factor)
        except Exception:
            # Keep fallback silent so benchmarking can proceed even if C backend is unavailable.
            pass

    return pseudo_inverse_dls(J, damping_factor=damping_factor) @ rhs


def rolling_arc_length(delta_alpha, radius):
    """
    Calculates the arc length rolled along the rim.

    Args:
        delta_alpha (float): The change in the rim contact angle (rad).
        radius (float): The wheel radius.

    Returns:
        float: The scalar rolling distance.
    """
    return delta_alpha * radius
