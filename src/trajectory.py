"""
Computing cartisian trajectory between two points in joint space.
"""
import roboticstoolbox as rtb
import numpy as np

from spatialmath import SE3
from denso_vp6242 import DensoVP6242


if __name__ == "__main__":
    denso = DensoVP6242()

    q0 = np.zeros(6)
    TE1 = denso.fkine(q0)
    TE2 = SE3.Trans(-0.5, 0, 0) @ SE3.Rz(np.pi) @ TE1

    t = np.arange(0, 2, 0.02)
    Ts = rtb.ctraj(TE1, TE2, t)

    qc = denso.ikine_LM(Ts, q0=q0) # Levenberg-Marquardt (LM) algorithm

    denso.plot(qc.q)
    rtb.xplot(t, denso.manipulability(qc.q), labels="manipulability")
    rtb.xplot(t, qc.q)
    rtb.xplot(t, Ts.t, labels="x y z")
    rtb.xplot(t, Ts.rpy("xyz"), labels="roll pitch yaw", block=True)
