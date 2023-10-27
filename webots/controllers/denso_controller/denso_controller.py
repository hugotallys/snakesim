"""denso_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import numpy as np
from controller import Robot  # type: ignore
from denso import DensoVP6242
from roboticstoolbox import ctraj
from spatialmath import SE3
from matplotlib import pyplot as plt


np.set_printoptions(precision=4, suppress=True)


class DensoRobot:

    def __init__(self) -> None:
        self.robot = Robot()
        self.denso = DensoVP6242()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.joints = [
            self.robot.getDevice(f'rotationalMotor{i}') for i in range(1, 7)]
        self.sensors = [
            self.robot.getDevice(f'positionSensor{i}') for i in range(1, 7)
        ]

    def step(self) -> int:
        return self.robot.step(self.timestep)

    def set_joint_position(self, q):
        for joint, q_i in zip(self.joints, q):
            joint.setPosition(q_i)


if __name__ == "__main__":
    denso_robot = DensoRobot()

    q0 = np.zeros(6)

    TE1 = denso_robot.denso.fkine(q0)
    TE2 = TE1  # SE3.Trans(-0.3, -0.1, -0.3) @ TE1

    dt = denso_robot.timestep / 1000
    t = np.arange(0, 5, dt)

    Ts = ctraj(TE1, TE2, t)

    T = denso_robot.denso.fkine(q0)

    man_values = np.zeros(len(t))
    pos_values = np.zeros((len(t), 3))
    joint_velocities = np.zeros((len(t), 6))

    qdot = np.zeros(6)

    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    i = 0
    while denso_robot.step() != -1:
        # Read the sensors:
        # Enter here functions to read sensor data, like:
        #  val = ds.getValue()
        # Process sensor data here.
        # Enter here functions to send actuator commands, like:
        #  motor.setPosition(10.0)

        J = denso_robot.denso.jacob0(q0, half="trans")
        J_pinv = np.linalg.pinv(J)
        x = T.t

        pos_values[i] = x

        dx = (Ts[i].t - x) / dt

        q0dot = 0.3 * np.array([0., 0, 0., 1., -1., 0.])
        # q0dot = denso_robot.denso.q0dot(q0, k0=100)

        qdot = J_pinv @ dx + (np.eye(6) - J_pinv @ J) @ q0dot
        qdot = qdot.reshape(6)

        joint_velocities[i] = qdot

        q0 = q0 + qdot * dt

        T = denso_robot.denso.fkine(q0)

        denso_robot.set_joint_position(q0)
        i = i + 1
        if i > len(t) - 1:
            break

        man_values[i] = denso_robot.denso.manipulability(q0)

    # plot the manipulability

    last_man = man_values[-1]
    plt.plot(t, man_values, linewidth=2)
    plt.xlabel("Time (s)")
    plt.ylabel("Manipulability")
    plt.title(f"Manipulability: {last_man:.4f}")
    plt.ylim(0, np.max(man_values) + 0.25 * np.max(man_values))
    plt.show()

    # plot the position

    des_pos_values = Ts.t

    plt.plot(t, pos_values[:, 0], label="x", linewidth=2)
    plt.plot(t, pos_values[:, 1], label="y", linewidth=2)
    plt.plot(t, pos_values[:, 2], label="z", linewidth=2)

    plt.plot(t, des_pos_values[:, 0], "--", label="x_des", linewidth=2)
    plt.plot(t, des_pos_values[:, 1], "--", label="y_des", linewidth=2)
    plt.plot(t, des_pos_values[:, 2], "--", label="z_des", linewidth=2)

    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title("Position")
    plt.show()

    # plot the joint velocities

    plt.plot(t, joint_velocities[:, 0], label="q1", linewidth=2)
    plt.plot(t, joint_velocities[:, 1], label="q2", linewidth=2)
    plt.plot(t, joint_velocities[:, 2], label="q3", linewidth=2)
    plt.plot(t, joint_velocities[:, 3], label="q4", linewidth=2)
    plt.plot(t, joint_velocities[:, 4], label="q5", linewidth=2)
    plt.plot(t, joint_velocities[:, 5], label="q6", linewidth=2)
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Joint Velocity (rad/s)")
    plt.title("Joint Velocity")
    plt.show()

    # Enter here exit cleanup code.
