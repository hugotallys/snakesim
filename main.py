"""Module to visualize densoVP6242 kinematic chain."""

import sys
import numpy as np

from roboticstoolbox import RevoluteDH, DHRobot

np.set_printoptions(precision=4, suppress=True)

if __name__ == "__main__":

    args = sys.argv[1:]

    if len(args) == 0:
        raise ValueError("Please specify a mode: teach or plot")

    pi2 = np.pi / 2

    D1 = 0.125
    D4 = 0.210
    D6 = 0.07
    A1 = 0.210
    A3 = -0.075

    dh_table = [
        RevoluteDH(d=D1,    a=0,    alpha=pi2,    offset=0),
        RevoluteDH(d=0,     a=A1,   alpha=0,      offset=pi2),
        RevoluteDH(d=0,     a=A3,   alpha=-pi2,   offset=-2*pi2),
        RevoluteDH(d=D4,    a=0,    alpha=pi2,    offset=0),
        RevoluteDH(d=0,     a=0,    alpha=-pi2,   offset=0),
        RevoluteDH(d=D6,    a=0,    alpha=0,      offset=0)
    ]

    robot = DHRobot(dh_table, name="Denso VP6242")

    print(robot)

    q = np.zeros(6)

    print("Forward kinematics on home position q = 0:")
    print(robot.fkine(q))

    print("Geometric Jacobian on home position q = 0:")
    print(robot.jacob0(q))

    print("Computing manipulability at home position q = 0:")
    man = np.array([10e3*robot.manipulability(q, method="yoshikawa")])
    print(f"{man[0]:.4f} x 10^-3")

    if args[0] == "teach":
        robot.teach(q, vellipse=True)
    else:
        robot.plot(q, block=True, vellipse=True)
