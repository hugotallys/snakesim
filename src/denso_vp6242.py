"""
Implements the Denso VP6242 robot.
"""
import numpy as np

from roboticstoolbox import RevoluteDH, DHRobot


class DensoVP6242(DHRobot): # pylint: disable=too-many-ancestors
    """
    Class that represents the Denso VP6242 robot.
    """
    def __init__(self):
        qlims = np.deg2rad(
            np.array([
                [-160, 160],
                [-120, 120],
                [  20, 160],
                [-160, 160],
                [-120, 120],
                [-360, 360]
            ])
        )
        dh_table = [
            RevoluteDH(d=0.125, a=0, alpha=np.pi/2, offset=0, qlim=qlims[0]),
            RevoluteDH(d=0, a=0.210, alpha=0, offset=np.pi/2, qlim=qlims[1]),
            RevoluteDH(d=0, a=-0.075, alpha=-np.pi/2, offset=-2*np.pi/2, qlim=qlims[2]),
            RevoluteDH(d=0.210, a=0, alpha=np.pi/2, offset=0, qlim=qlims[3]),
            RevoluteDH(d=0, a=0, alpha=-np.pi/2, offset=0, qlim=qlims[4]),
            RevoluteDH(d=0.07, a=0, alpha=0, offset=0, qlim=qlims[5])
        ]
        super().__init__(dh_table, name="Denso VP6242")
