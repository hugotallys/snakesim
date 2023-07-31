"""Module to visualize densoVP6242 kinematic chain."""

import sys
import numpy as np

from denso_vp6242 import DensoVP6242

np.set_printoptions(precision=4, suppress=True)

if __name__ == "__main__":

    args = sys.argv[1:]

    if len(args) == 0:
        raise ValueError("Please specify a mode: `teach` or `plot`")

    robot = DensoVP6242()

    print(robot)

    q = np.zeros(6)

    print(f"Forward kinematics joint position q = {q}:")
    print(robot.fkine(q))

    if args[0] == "teach":
        robot.teach(q)
    elif args[0] == "plot":
        robot.plot(q, block=True)
    else:
        raise ValueError("Please specify a valid mode: `teach` or `plot`")
