import numpy as np
import matplotlib.pyplot as plt

from roboticstoolbox import DHRobot, RevoluteDH, ctraj
from spatialmath import SE3


class PlanarRoboticArm:
    def __init__(self, arm_lengths):
        self.arm_lengths = arm_lengths
        self.N = len(arm_lengths)
        self.robot = DHRobot([
            RevoluteDH(d=0, a=length, alpha=0) for length in self.arm_lengths
        ], name=f"Planar {self.arm_lengths}R Robot")
        self.qlims = [
            (-np.pi, np.pi) for length in self.arm_lengths
        ]

    def jacobian(self, q):
        return self.robot.jacob0(q, half="trans")[:2, :]

    def q0dot(self, q, k0=1.0, objective="manipulability"):
        if objective == "manipulability":
            return k0 * self.gradient(self.manipulability, q)
        elif objective == "joint_limit_distance":
            n = len(q)
            values = []
            for i in range(len(self.qlims)):
                q_min, q_max = self.qlims[i][0], self.qlims[i][1]
                q_mean = 0.5*(q_max + q_min)
                values.append(
                    (q[i] - q_mean) / (q_max - q_min)**2
                )
            return (-k0 / n) * (np.array(values)).reshape(n, 1)
        elif objective == "sincos":
            return k0 * np.array(
                [0., 2*np.sin(q[1])*np.cos(q[1]), 2*np.sin(q[2])*np.cos(q[2])]
            )
        else:
            raise ValueError(
                "Invalid objective. Must be either"
                " 'manipulability', 'joint_limit_distance' or 'sincos'"
            )

    def joint_distance(self, q):
        """
        Computes the distance from mechanical joint limits.
        """
        n = len(q)
        values = []
        for i in range(len(self.qlims)):
            q_min, q_max = self.qlims[i][0], self.qlims[i][1]
            q_mean = 0.5*(q_max + q_min)
            values.append(
                ((q[i] - q_mean) / (q_max - q_min))**2
            )
        return (-1/(2*n)) * np.sum(values)

    def manipulability(self, q):
        jac = self.jacobian(q)
        return np.sqrt(np.linalg.det(jac @ jac.T))

    @staticmethod
    def gradient(f, x, h=0.01):
        grad = np.zeros_like(x)
        for i in range(len(x)):
            xp, xm = x.copy(), x.copy()
            xp[i] = xp[i] + h
            xm[i] = xm[i] - h
            grad[i] = (f(xp) - f(xm)) / (2 * h)
        return grad


def main():
    arm_lengths = np.ones(20)

    dt = 0.1
    total_time = 5
    t = np.arange(0, total_time, dt)

    K = len(t)  # Number of time steps
    N = len(arm_lengths)  # Number of joints

    k0_values = np.arange(0, 50, 10)
    alpha = 0.1
    obj_function = "joint_limit_distance"

    plot_criteria = {
        "manipulability": "Manipulability",
        "joint_limit_distance": "Distance to Joint Limits",
        "sincos": "Approximation of Manipulability"
    }

    # craete a 1x2 figure
    fig, ax = plt.subplots(1, 2, figsize=(12, 6))

    ax[0].set_title(plot_criteria[obj_function])
    ax[0].set_xlabel("Time (s)")

    ax[1].set_title("Integral of " + plot_criteria[obj_function])
    ax[1].set_xlabel("Time (s)")

    for k0 in k0_values:

        q_values = np.zeros((K, N))
        qdot_values = np.zeros((K, N))
        x_values = np.zeros((K, 2))
        obj_function_values = np.zeros(K)
        int_obj_function_values = np.zeros(K)

        planar_arm = PlanarRoboticArm(arm_lengths)

        # initial configuration
        aone = np.ones(N)
        aone[1::2] = -1
        q_values[0, :] = aone * np.pi / 2

        TE1 = planar_arm.robot.fkine(q_values[0, :])
        TE2 = SE3.Trans(-20, -20, 0) @ TE1

        Ts = ctraj(TE1, TE2, t)

        T = TE1.t
        x_values[0, :] = t[:2]

        if obj_function == "manipulability" or obj_function == "sincos":
            obj_function_values[0] = planar_arm.manipulability(
                q_values[0, :])
            int_obj_function_values[0] = planar_arm.manipulability(
                q_values[0, :]) * dt
        elif obj_function == "joint_limit_distance":
            obj_function_values[0] = planar_arm.joint_distance(
                q_values[0, :])
            int_obj_function_values[0] = planar_arm.joint_distance(
                q_values[0, :]) * dt

        for i in range(1, K):
            dx = (Ts[i].t[:2] - t[:2]) / dt
            J = planar_arm.jacobian(q_values[i - 1, :])
            JT = np.linalg.pinv(J)

            q0dot = np.squeeze(planar_arm.q0dot(
                q_values[i - 1, :], k0=k0, objective=obj_function))
            qdot = alpha * JT @ dx + (np.eye(N) - JT @ J) @ q0dot

            qdot_values[i, :] = qdot
            q_values[i, :] = q_values[i - 1, :] + dt * qdot

            T = planar_arm.robot.fkine(q_values[i, :]).t

            x_values[i, :] = T[:2]

            if obj_function == "manipulability" or obj_function == "sincos":
                obj_function_values[i] = planar_arm.manipulability(
                    q_values[i, :])
                int_obj_function_values[i] = int_obj_function_values[i - 1] + \
                    planar_arm.manipulability(q_values[i, :]) * dt
            elif obj_function == "joint_limit_distance":
                obj_function_values[i] = planar_arm.joint_distance(
                    q_values[i, :])
                int_obj_function_values[i] = int_obj_function_values[i - 1] + \
                    planar_arm.joint_distance(q_values[i, :]) * dt

        qdot_values[0, :] = qdot_values[1, :]

        ax[0].plot(
            t, obj_function_values, "-", linewidth=2,
            label=f"k0 = {k0:.2f}")

        ax[1].plot(
            t, int_obj_function_values, "-", linewidth=2,
            label=f"k0 = {k0:.2f}")

    ax[0].legend()
    ax[1].legend()

    plt.show()


if __name__ == "__main__":
    main()
