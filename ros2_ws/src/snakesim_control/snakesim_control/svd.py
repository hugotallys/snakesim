import matplotlib.pyplot as plt
import numpy as np

np.set_printoptions(precision=5, suppress=True)


def s(x): return np.sin(x)
def c(x): return np.cos(x)


def pretty_print(matrix):
    print(np.array_str(matrix, precision=3, suppress_small=True))


def jacob(q):
    q1, q2, q3 = q
    return np.array([
        -s(q1)-s(q1+q2)-s(q1+q2+q3),  -s(q1+q2)-s(q1+q2+q3),  -s(q1+q2+q3),
        c(q1) + c(q1+q2)+c(q1+q2+q3), c(q1+q2)+c(q1+q2+q3), c(q1+q2+q3)
    ]).reshape(2, 3)


def plot_3r(q, P, ax, qdot=None, hideLinks=False):
    J = jacob(q)

    # Applies the linear transformation to the points
    jP = J @ P

    # Computes the end effector position

    x1, y1 = c(q[0]), s(q[0])
    x2, y2 = x1 + c(q[0] + q[1]), y1 + s(q[0] + q[1])
    x3, y3 = x2 + c(q[0] + q[1] + q[2]), y2 + s(q[0] + q[1] + q[2])

    if not hideLinks:
        ax.plot([0, x1, x2, x3], [0, y1, y2, y3], 'o-', color='black')
        ax.set_yticklabels([])
        ax.set_xticklabels([])
        ef = np.array([[x3], [y3]])
        jP = jP + ef
    else:
        ef = np.zeros(2).reshape(2, 1)
        ax.set_ylabel('$\\dot{y}$')
        ax.set_xlabel('$\\dot{x}$')
        ax.set_title('$\\xi = J \\dot{q}$')

    ax.plot(jP[0, :], jP[1, :], '-', color='blue', alpha=0.3)

    if qdot is not None:
        xi_dot = (J @ qdot).reshape(2, 1) + ef
        ax.plot(
            [ef[0][0], xi_dot[0][0]], [ef[1][0], xi_dot[1][0]],
            '-', color='red'
        )

    ax.axis('equal')
    ax.grid(True)

    return J


if __name__ == '__main__':

    # Plots the transformed points in the same figure
    fig, ax = plt.subplots()

    # Defines the jacobian for a given configuration
    # q = np.array([0.5, 1.0, 1.5]) * (np.pi / 4)
    q = np.deg2rad([60., -60., -60.])
    qs = [
        1.0 * q,
        0.5 * q,
        0.1 * q
    ]

    # creates points in a unit sphere
    theta = np.linspace(0, np.pi, 100)
    phi = np.linspace(0, 2*np.pi, 100)

    x = np.outer(np.sin(theta), np.cos(phi))
    y = np.outer(np.sin(theta), np.sin(phi))
    z = np.outer(np.cos(theta), np.ones_like(phi))

    # creates a matrix with the points
    P = np.vstack((x.flatten(), y.flatten(), z.flatten()))

    for q in qs:
        plot_3r(q, P, ax)

    # # Plots the unit norm joint velocities

    fig, ax = plt.subplots(1, 2)

    qdot = np.array([1.0, 0., 0.])

    ax[0].plot(P[0, :], P[1, :], '-', color='blue', alpha=0.3)

    ax[0].plot([0, qdot[0]], [0, qdot[1]], '-', color='red')

    ax[0].axis('equal')
    ax[0].grid(True)

    ax[0].set_title('$||\\dot{q}||^2 = 1$')
    ax[0].set_xlabel('$\\dot{q_1}$')
    ax[0].set_ylabel('$\\dot{q_2}$')

    J = plot_3r(qs[1], P, ax[1], qdot, True)

    # Computes the SVD of J

    U, S, VT = np.linalg.svd(J)

    UT = U.T
    V = VT.T

    S = S.tolist() + [0.]
    S = np.diag(S)[0:2, :]

    # Plots the transformed points by V

    pV = VT @ P

    qdot_v = (VT @ qdot).reshape(3, 1)

    fig, ax = plt.subplots(1, 3)

    ax[0].plot(pV[0, :], pV[1, :], '-', color='blue', alpha=0.3)

    ax[0].axis('equal')
    ax[0].grid(True)
    ax[0].set_title('Transfomação $V^\\top$')

    # Plots the transformed points by S

    pS = S @ pV

    ax[1].plot(pS[0, :], pS[1, :], '-', color='blue', alpha=0.3)
    ax[1].plot([0, S[0][0]], [0, 0], '-', color='blue')
    ax[1].plot([0, 0], [0, S[1][1]], '-', color='red')

    ax[1].axis('equal')
    ax[1].grid(True)
    ax[1].set_title('Transformação $\\Sigma$')

    # Plots the transformed points by U

    pU = U @ pS

    u1, u2 = S[0][0] * U[:, 0], S[1][1] * U[:, 1]

    ax[2].plot(pU[0, :], pU[1, :], '-', color='blue', alpha=0.3)
    ax[2].plot([0, u1[0]], [0, u1[1]], '-', color='blue')
    ax[2].plot([0, u2[0]], [0, u2[1]], '-', color='red')

    ax[2].axis('equal')
    ax[2].grid(True)
    ax[2].set_title('Transformação $U$')

    plt.show()
