import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH


class Robot:
    def __init__(self):

        self.qlim = [-np.pi, np.pi]

        self.robot = DHRobot(
            [
                RevoluteDH(d=0, a=0.06, alpha=np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=-np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=-np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.06, alpha=-np.pi / 2, qlim=self.qlim),
                RevoluteDH(d=0, a=0.02, alpha=np.pi / 2, qlim=self.qlim),
            ],
            name="Snake",
        )

    def jacobian(self, q):
        return self.robot.jacob0(q, half="trans")[:3, :]

    def q0dot(self, q, k0=0.0):
        n = len(q)
        values = []
        qlims = self.robot.qlim.T
        for i in range(qlims.shape[0]):
            q_min, q_max = qlims[i][0], qlims[i][1]
            q_mean = 0.5 * (q_max + q_min)
            values.append((q[i] - q_mean) / (q_max - q_min) ** 2)
        return (-k0 / n) * (np.array(values)).reshape(n, 1)

    def joint_distance(self, q):
        n = len(q)
        values = []
        qlims = self.robot.qlim.T
        for i in range(qlims.shape[0]):
            q_min, q_max = qlims[i][0], qlims[i][1]
            q_mean = 0.5 * (q_max + q_min)
            values.append(((q[i] - q_mean) / (q_max - q_min)) ** 2)
        return (-1.0 / (2 * n)) * np.sum(values)
