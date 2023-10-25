import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from roboticstoolbox import DHRobot, RevoluteDH, ctraj
from spatialmath import SE3
import matplotlib.patches as patches


class PlanarRoboticArmAnimation:
    def __init__(self, arm_lengths):
        self.arm_lengths = arm_lengths
        self.N = len(arm_lengths)
        self.fig, self.ax = plt.subplots()
        self.robot = DHRobot([
            RevoluteDH(d=0, a=length, alpha=0) for length in self.arm_lengths
        ], name=f"Planar {self.arm_lengths}R Robot")
        self.qlims = [
            (-np.pi, np.pi) for length in self.arm_lengths
        ]

    def animate_pose(self, q, target, path, ellipse, angle):
        self.ax.clear()
        self.plot_pose(
            self.ax, self.arm_lengths, q, target, path, ellipse, angle)
        self.ax.set_xlim(-sum(self.arm_lengths), sum(self.arm_lengths))
        self.ax.set_ylim(-sum(self.arm_lengths), sum(self.arm_lengths))
        self.ax.set_aspect('equal')

    def plot_pose(self, ax, al, q, target, path, ellipse, angle):
        A = self.robot.fkine_all(q)
        ax.plot(A.t[:, 0], A.t[:, 1], "b-", linewidth=4)
        ax.plot(A.t[:, 0], A.t[:, 1], "ko", markersize=5)
        ax.plot(A[-1].t[0], A[-1].t[1], "ro", markersize=5)
        ax.plot(target[0], target[1], "go", markersize=5)
        ax.plot(path[:, 0], path[:, 1], "g-", linewidth=1)
        ellipse = patches.Ellipse(
            (A[-1].t[0], A[-1].t[1]), width=ellipse[0],
            height=ellipse[1], fill=False, color='b',
            angle=np.degrees(angle))
        # Add the ellipse to the axis
        ax.add_patch(ellipse)

    def J(self, q):
        return self.robot.jacob0(q, half="trans")[:2, :]

    def q0dot(self, q, k0=1.0, objective="manipulability"):
        if objective == "manipulability":
            return k0 * np.array(
                [0., 2*np.sin(q[1])*np.cos(q[1]), 2*np.sin(q[2])*np.cos(q[2])]
            )
        elif objective == "joint_range":
            n = len(q)
            values = []
            for i in range(len(self.qlims)):
                q_min, q_max = self.qlims[i][0], self.qlims[i][1]
                q_mean = 0.5*(q_max + q_min)
                values.append(
                    (q[i] - q_mean) / (q_max - q_min)**2
                )
            return (k0 / n) * (np.array(values)).reshape(n, 1)
        else:
            raise ValueError(
                "Invalid objective. Must be either"
                " 'manipulability' or 'joint_range'"
            )

    @staticmethod
    def compute_angle(eigenvectors):
        # Calculate the angle of the semi-major axis
        # with respect to the x-axis
        angle = np.arctan2(
            eigenvectors[1, 0], eigenvectors[0, 0])
        return angle


def main(gif_filename=None):
    # Define the arm lengths and configurations
    arm_lengths = np.ones(3)  # Example arm lengths
    K = 100  # Number of configurations
    N = len(arm_lengths)  # Number of joints

    # Generate some example configurations
    q_values = np.zeros((K, N))
    qdot_values = np.zeros((K, N))
    x_values = np.zeros((K, 2))
    man_values = np.zeros(K)
    angle_values = np.zeros(K)
    axis_values = np.zeros((K, 2))

    # Create the animation object
    arm_animation = PlanarRoboticArmAnimation(
        arm_lengths)

    # alternating vector of 1 and -1 and size N
    aone = np.ones(N)
    aone[1::2] = -1

    q_values[0, :] = aone * np.pi / 2

    TE1 = arm_animation.robot.fkine(q_values[0, :])
    TE2 = SE3.Trans(-2, -3, 0) @ TE1

    Ts = ctraj(TE1, TE2, K)

    t = TE1.t
    x_values[0, :] = t[:2]
    for i in range(1, K):
        dx = Ts[i].t[:2] - t[:2]
        J = arm_animation.J(q_values[i - 1, :])
        M = J @ J.T
        eigenvalues, eigenvectors = np.linalg.eig(M)
        # Calculate the lengths of the semi-axes
        axis_values[i, :] = np.sqrt(eigenvalues)
        JT = np.linalg.pinv(J)
        q0dot = np.squeeze(arm_animation.q0dot(
            q_values[i - 1, :], k0=0, objective="manipulability"))
        # q0dot = np.squeeze(arm_animation.q0dot(
        # q_values[i - 1, :], k0=-10, objective="joint_range"))
        qdot = JT @ dx + (np.eye(len(arm_lengths)) - JT @ J) @ q0dot
        qdot_values[i, :] = qdot
        q_values[i, :] = q_values[i - 1, :] + qdot
        T = arm_animation.robot.fkine(q_values[i, :])
        t = T.t
        angle_values[i] = arm_animation.compute_angle(eigenvectors)
        x_values[i, :] = t[:2]
        man_values[i] = np.sqrt(np.linalg.det(M))

    man_values[0] = man_values[1]
    qdot_values[0, :] = qdot_values[1, :]

    def update(frame):
        arm_animation.animate_pose(
            q_values[frame], TE2.t[:2], x_values[:frame+1, :],
            axis_values[frame, :], angle_values[frame])

    desired_frame_rate = 60  # Desired frame rate in frames per second
    interval = 1000 / desired_frame_rate  # Interval in milliseconds

    # Create the animation
    anim = FuncAnimation(
        arm_animation.fig, update, frames=K,
        repeat=False, blit=False, interval=interval
    )

    if gif_filename is not None:
        # Use the PillowWriter to save the animation as a GIF
        anim.save(gif_filename, writer=PillowWriter(fps=desired_frame_rate))

    plt.show()

    # plot joint angles
    plt.title("Joint Position")
    plt.plot(
        q_values, "-", linewidth=2,
        label=[f"q{i}" for i in range(1, len(arm_lengths)+1)])
    plt.legend()

    plt.figure()
    # plot joint velocities
    plt.title("Joint Velocity")
    plt.plot(
        qdot_values, "-", linewidth=2,
        label=[f"q{i}" for i in range(1, len(arm_lengths)+1)])
    plt.legend()

    plt.figure()
    # plot manipulability
    plt.title("Manipluability")
    plt.plot(
        man_values, "-", linewidth=2)
    plt.ylim(0, np.max(man_values) + 0.25 * np.max(man_values))
    plt.show()

    print("Final manipulability = %f" % (man_values[-1],))
    print("Minimal value for manipulability = %f" % (man_values.min(),))


if __name__ == "__main__":
    filename = None  # "robotic_arm_animation.gif"
    main(gif_filename=filename)
