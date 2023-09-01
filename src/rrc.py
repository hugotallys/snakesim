"""
Computing cartisian trajectory between two points in joint space.
"""
import roboticstoolbox as rtb
import numpy as np

from spatialmath import SE3
from denso_vp6242 import DensoVP6242


def vec_vel(T):
    """ Given a homogenous transformation matrix, return a 6-vector with velocities"""
    return np.concatenate([T.t, T.rpy("xyz")])


if __name__ == "__main__":
    denso = DensoVP6242()

    q0 = np.zeros(6)

    print(q0)

    TE1 = denso.fkine(q0)
    TE2 = SE3.Trans(-0.05, 0, 0) @ SE3.Rz(np.pi) @ TE1

    dt = 0.02
    t = np.arange(0, 2, dt)
    Ts = rtb.ctraj(TE1, TE2, t)

    q = np.zeros((len(t), 6))

    X = np.zeros((len(t), 6))

    for i in range(2, len(t)):
        J = denso.jacob0(q0)
        dx = (Ts[i].t - Ts[i-1].t) / dt
        row, pitch, yaw = Ts[i].rpy("xyz")
        dx = np.concatenate([dx, [row, pitch, yaw]]).reshape(6, 1)
        qdot = np.linalg.pinv(J) @ dx
        qdot = qdot.reshape(6)
        q0 = q0 + qdot * dt
        q[i] = q0

        X[i] = vec_vel(denso.fkine(q0))

    denso.plot(q)


    # --- Joint trajectory --- #
    rtb.xplot(t, q, labels="q1 q2 q3 q4 q5 q6")

    # --- Manipulability --- #
    rtb.xplot(t, denso.manipulability(q), labels="manipulability")

    # --- Cartesian trajectory --- #
    rtb.xplot(t, Ts.t, labels="x y z")
    rtb.xplot(t, Ts.rpy("xyz"), labels="roll pitch yaw", block=True)

    print("Final pose:")
    print(TE2)

    print("Final achieved pose:")
    print(denso.fkine(q[-1]))

    # --- Plotting the cartesian trajectory performed --- #
    rtb.xplot(t, X[:, 0:3], labels="x y z")
    rtb.xplot(t, X[:, 3:6], labels="roll pitch yaw", block=True)
