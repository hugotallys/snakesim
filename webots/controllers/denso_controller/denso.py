"""
Implements the Denso VP6242 robot.
"""
import numpy as np

from roboticstoolbox import RevoluteDH, DHRobot


class DensoVP6242(DHRobot):  # pylint: disable=too-many-ancestors
    """
    Class that represents the Denso VP6242 robot.
    """
    def __init__(self):
        self.qlims = np.deg2rad(
            np.array([
                [-160, 160],
                [-120, 120],
                [   0, 160], # [  20, 160],
                [-160, 160],
                [-120, 120],
                [-360, 360]
            ])
        )
        dh_table = [
            RevoluteDH(d=0.125 + 0.155, a=0,        alpha=np.pi/2,  offset=0,
                       qlim=self.qlims[0]),
            RevoluteDH(d=0,     a=0.210,    alpha=0,        offset=np.pi/2,
                       qlim=self.qlims[1]),
            RevoluteDH(d=0,     a=-0.075,   alpha=-np.pi/2, offset=-2*np.pi/2,
                       qlim=self.qlims[2]),
            RevoluteDH(d=0.210, a=0,        alpha=np.pi/2,  offset=0,
                       qlim=self.qlims[3]),
            RevoluteDH(d=0,     a=0,        alpha=-np.pi/2, offset=0,
                       qlim=self.qlims[4]),
            RevoluteDH(d=0.07, a=0,         alpha=0,        offset=0,
                       qlim=self.qlims[5])
        ]
        super().__init__(dh_table, name="Denso VP6242")

    def q0dot(self, q, k0=1.0):
        n = len(q)
        values = []
        for i in range(len(self.qlims)):
            q_min, q_max = self.qlims[i][0], self.qlims[i][1]
            q_mean = 0.5*(q_max + q_min)
            values.append(
                (q[i] - q_mean) / (q_max - q_min)**2
            )
        return np.squeeze((-k0 / n) * (np.array(values)))
