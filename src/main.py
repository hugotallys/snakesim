"""Module to visualize densoVP6242 kinematic chain."""

import sys
import numpy as np

from denso_vp6242 import DensoVP6242, LInspector

np.set_printoptions(precision=4, suppress=True)

if __name__ == "__main__":

    args = sys.argv[1:]

    if len(args) == 0:
        raise ValueError("Please specify a mode: `teach` or `plot`")

    robot = LInspector() # DensoVP6242()

    print(robot)

    q = np.zeros(4)

    print(f"Forward kinematics joint position q = {q}:")
    print(robot.fkine(q))

    qs = np.zeros((100, 4))

    qs[:,0] = np.linspace(-np.pi, np.pi, 100)
    qs[:,1] = np.linspace(0, 1, 100)
    qs[:,2] = np.linspace(0, 1, 100)
    qs[:,3] = np.linspace(-np.pi, np.pi, 100)

    if args[0] == "teach":
        robot.teach(q)
    elif args[0] == "plot":
        robot.plot(qs, block=True)
    else:
        raise ValueError("Please specify a valid mode: `teach` or `plot`")
