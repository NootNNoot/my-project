#!/usr/bin/env python3

import math
import numpy as np
import pinocchio as pin
from scipy.linalg import solve_discrete_are


URDF_PATH = "/home/kaper/double_pend_ws/src/double_pendulum_bringup/config/double_pendulum_static.urdf"

ALL_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "pendulum_joint_2",
]

CONTROL_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
]

Q_NOMINAL_DICT = {
    "joint1": 0.0,
    "joint2": 0.79,
    "joint3": 0.79,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": 0.0,
    "pendulum_joint_2": 0.0,
}

DT = 0.01
FD_EPS_X = 1e-4
FD_EPS_U = 1e-3

OUT_PATH = "lqr_gain.npy"


def joint_q_index(model, joint_name):
    jid = model.getJointId(joint_name)
    if jid == 0 or jid >= len(model.joints):
        raise RuntimeError(f"Joint not found in Pinocchio model: {joint_name}")
    return model.joints[jid].idx_q


def joint_v_index(model, joint_name):
    jid = model.getJointId(joint_name)
    if jid == 0 or jid >= len(model.joints):
        raise RuntimeError(f"Joint not found in Pinocchio model: {joint_name}")
    return model.joints[jid].idx_v

def joint_indices(model, joint_name):
    jid = model.getJointId(joint_name)
    if jid == 0 or jid >= len(model.joints):
        raise RuntimeError(f"Joint not found: {joint_name}")

    j = model.joints[jid]
    return j.idx_q, j.nq, j.idx_v, j.nv


def set_joint_angle(model, q, joint_name, angle):
    qidx, nq, _, _ = joint_indices(model, joint_name)

    if nq == 1:
        q[qidx] = angle
    elif nq == 2:
        q[qidx] = math.cos(angle)
        q[qidx + 1] = math.sin(angle)
    else:
        raise RuntimeError(f"Unsupported joint nq={nq} for {joint_name}")


def get_joint_angle(model, q, joint_name):
    qidx, nq, _, _ = joint_indices(model, joint_name)

    if nq == 1:
        return q[qidx]
    elif nq == 2:
        return math.atan2(q[qidx + 1], q[qidx])
    else:
        raise RuntimeError(f"Unsupported joint nq={nq} for {joint_name}")


def set_joint_velocity(model, v, joint_name, vel):
    _, _, vidx, nv = joint_indices(model, joint_name)

    if nv != 1:
        raise RuntimeError(f"Unsupported joint nv={nv} for {joint_name}")

    v[vidx] = vel


def get_joint_velocity(model, v, joint_name):
    _, _, vidx, nv = joint_indices(model, joint_name)

    if nv != 1:
        raise RuntimeError(f"Unsupported joint nv={nv} for {joint_name}")

    return v[vidx]


def wrap_to_pi(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def build_model():
    model = pin.buildModelFromUrdf(URDF_PATH)
    data = model.createData()
    print(model)
    print(f"nq={model.nq}, nv={model.nv}")
    return model, data


def make_q_nominal(model):
    q = pin.neutral(model)

    for name, value in Q_NOMINAL_DICT.items():
        set_joint_angle(model, q, name, value)

    return q


def state_to_qv(model, x, q_nom):
    n = len(ALL_JOINTS)

    q = q_nom.copy()
    v = np.zeros(model.nv)

    for i, name in enumerate(ALL_JOINTS):
        nominal_angle = get_joint_angle(model, q_nom, name)
        set_joint_angle(model, q, name, nominal_angle + x[i])
        set_joint_velocity(model, v, name, x[n + i])

    return q, v


def qv_to_state(model, q, v, q_nom):
    n = len(ALL_JOINTS)
    x = np.zeros(2 * n)

    for i, name in enumerate(ALL_JOINTS):
        angle = get_joint_angle(model, q, name)
        nominal_angle = get_joint_angle(model, q_nom, name)

        x[i] = wrap_to_pi(angle - nominal_angle)
        x[n + i] = get_joint_velocity(model, v, name)

    return x


def control_to_tau(model, u):
    tau = np.zeros(model.nv)

    for i, name in enumerate(CONTROL_JOINTS):
        _, _, vidx, nv = joint_indices(model, name)

        if nv != 1:
            raise RuntimeError(f"Unsupported control joint nv={nv} for {name}")

        tau[vidx] = u[i]

    return tau


def dynamics_step(model, data, q_nom, x, u):
    q, v = state_to_qv(model, x, q_nom)
    tau = control_to_tau(model, u)

    # Passive joints get zero torque automatically.
    a = pin.aba(model, data, q, v, tau)

    v_next = v + DT * a
    q_next = pin.integrate(model, q, DT * v_next)

    return qv_to_state(model, q_next, v_next, q_nom)


def linearize(model, data, q_nom):
    nx = 2 * len(ALL_JOINTS)
    nu = len(CONTROL_JOINTS)

    x0 = np.zeros(nx)
    u0 = np.zeros(nu)

    A = np.zeros((nx, nx))
    B = np.zeros((nx, nu))

    f0 = dynamics_step(model, data, q_nom, x0, u0)

    for i in range(nx):
        dx = np.zeros(nx)
        dx[i] = FD_EPS_X

        fp = dynamics_step(model, data, q_nom, x0 + dx, u0)
        fm = dynamics_step(model, data, q_nom, x0 - dx, u0)

        A[:, i] = (fp - fm) / (2.0 * FD_EPS_X)

    for j in range(nu):
        du = np.zeros(nu)
        du[j] = FD_EPS_U

        fp = dynamics_step(model, data, q_nom, x0, u0 + du)
        fm = dynamics_step(model, data, q_nom, x0, u0 - du)

        B[:, j] = (fp - fm) / (2.0 * FD_EPS_U)
    
    A = A

    return A, B, f0


def solve_lqr(A, B):
    nx = A.shape[0]

    Q = np.eye(nx)

    Q[0, 0] = 1.0
    Q[1, 1] = 4.0
    Q[2, 2] = 4.0
    Q[3, 3] = 1.0
    Q[4, 4] = 1.0
    Q[5, 5] = 5.0
    Q[6, 6] = 30.0

    Q[7, 7] = 0.5
    Q[8, 8] = 1.0
    Q[9, 9] = 1.0
    Q[10, 10] = 0.5
    Q[11, 11] = 0.5
    Q[12, 12] = 3.0
    Q[13, 13] = 8.0

    R_base = np.diag([5.0, 2.0, 2.0, 8.0, 8.0, 5.0])

    for shrink in [1.0, 0.995, 0.99, 0.98, 0.95]:
        for rscale in [1.0, 2.0, 5.0, 10.0, 25.0, 50.0]:
            try:
                A_try = shrink * A
                R = rscale * R_base

                P = solve_discrete_are(A_try, B, Q, R)
                K = np.linalg.solve(B.T @ P @ B + R, B.T @ P @ A_try)

                eigs = np.linalg.eigvals(A_try - B @ K)
                print(f"Solved DARE with shrink={shrink}, rscale={rscale}")
                print(f"max closed-loop eig abs={np.max(np.abs(eigs)):.6f}")

                return K

            except Exception as ex:
                print(f"DARE failed: shrink={shrink}, rscale={rscale}: {ex}")

    raise RuntimeError("Could not solve LQR after all fallback attempts.")


def main():
    model, data = build_model()
    q_nom = make_q_nominal(model)

    print("Nominal q:")
    for name in ALL_JOINTS:
        print(f"  {name}: {get_joint_angle(model, q_nom, name): .4f}")

    A, B, f0 = linearize(model, data, q_nom)

    print(f"A shape: {A.shape}")
    print(f"B shape: {B.shape}")
    print(f"f0 norm: {np.linalg.norm(f0):.6f}")

    K = solve_lqr(A, B)

    print(f"K shape: {K.shape}")
    print(K)

    np.save(OUT_PATH, K)
    print(f"Saved K to {OUT_PATH}")


if __name__ == "__main__":
    main()