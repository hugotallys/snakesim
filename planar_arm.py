import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
from roboticstoolbox import DHRobot, RevoluteDH


class PlanarArmEllipsoid:
    def __init__(self, arm_lengths):
        self.arm_lengths = arm_lengths
        self.N = len(arm_lengths)
        self.fig, self.ax = plt.subplots()
        self.robot = DHRobot(
            [
                RevoluteDH(d=0, a=length, alpha=0, qlim=(-np.pi, np.pi))
                for length in self.arm_lengths
            ],
            name=f"Planar {self.arm_lengths}R Robot",
        )

    def animate_pose(self, q):
        self.ax.clear()

        J = self.robot.jacob0(q, half="trans")[:2, :]

        U, S, _ = np.linalg.svd(J)

        # Calculate the lengths of the semi-axes
        ellipse_axis = S
        ellipse_angle = np.arctan2(U[1, 0], U[0, 0])

        A = self.robot.fkine_all(q)
        self.ax.plot(A.t[:, 0], A.t[:, 1], "b-", linewidth=4)
        self.ax.plot(A.t[:, 0], A.t[:, 1], "ko", markersize=5)
        self.ax.plot(A[-1].t[0], A[-1].t[1], "ro", markersize=5)
        ellipse = patches.Ellipse(
            (A[-1].t[0], A[-1].t[1]),
            width=ellipse_axis[0],
            height=ellipse_axis[1],
            fill=False,
            color="b",
            angle=np.degrees(ellipse_angle),
        )
        # Add the ellipse to the axis
        self.ax.add_patch(ellipse)

        arm_lim = sum(self.arm_lengths)

        self.ax.set_xlim(-arm_lim, arm_lim)
        self.ax.set_ylim(-arm_lim, arm_lim)
        self.ax.set_aspect("equal")

        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")

        self.ax.set_title(f"Manipulibility Ellipsoid - Planar {self.N}R Robot")

        self.ax.text(
            arm_lim + 0.25,
            0.0,
            rf"$\mu={np.prod(S):.2f}$",
            size=16,
        )

    def update(self, val):
        q = np.array([slider.val for slider in self.sliders])
        self.animate_pose(q)

    def teach(self, q):
        self.animate_pose(q)

        self.sliders = []

        for i in range(self.N):
            ax_slider = plt.axes(
                [0.05, i * ((0.75 - 0.25) / self.N) + 0.25, 0.15, 0.05]
            )
            slider = Slider(
                ax=ax_slider,
                label=f"Joint {i+1}",
                valmin=-np.pi,
                valmax=np.pi,
                valinit=q[i],
            )

            self.sliders.append(slider)

            slider.on_changed(self.update)

        plt.show()


def main():
    arm_lengths = np.ones(3)
    robot_arm = PlanarArmEllipsoid(arm_lengths)
    q = np.array([0.5 for _ in arm_lengths])
    robot_arm.teach(q)


if __name__ == "__main__":
    main()
