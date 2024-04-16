import os
import re
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def get_params(filename):
    match = re.search(r"gain=([0-9.]+)metric=([a-zA-Z_]+)", filename)
    if match:
        return match.groups()
    else:
        raise ValueError("Filename does not contain the expected format.")


def get_data(directory, filename):
    if filename.endswith(".csv"):
        gain, metric = get_params(filename)
        df = pd.read_csv(os.path.join(directory, filename))
        df["time"] = 0.032 * df.index
        df = df.drop(df.index[0])
        return df, gain, metric
    return None


def compute_position_error(row, last_row):
    position = np.array([row["x"], row["y"], row["z"]])
    desired_posisition = np.array(
        [last_row["x_"], last_row["y_"], last_row["z_"]]
    )
    return np.linalg.norm(position - desired_posisition)


def plot_csv_files(directory, show=False):
    metric = None
    gains = []
    dfs = []

    for filename in os.listdir(directory):
        if filename.endswith(".png"):
            return

    for filename in os.listdir(directory):
        data = get_data(directory, filename)
        if data is not None:
            df, gain, metric = data
            gains.append(gain)
            dfs.append(df)

    if metric is not None:

        # Plot #1 - Metric value over time for different gains

        fig, ax = plt.subplots(1, 1, figsize=(10, 5))

        for df, gain in zip(dfs, gains):
            ax.plot(
                df["time"].values[1:],
                abs(df["metric"].values[1:]),
                label=gain,
            )

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Metric")
        ax.legend()

        plt.savefig(f"{directory}/metric_{metric}.png")

        # Plot #2 - Position error over time for different gains

        fig, ax = plt.subplots(1, 1, figsize=(10, 5))

        for df, gain in zip(dfs, gains):
            df["position_error"] = df.apply(
                lambda row: compute_position_error(row, df.iloc[-1]),
                axis=1,
            )
            ax.plot(
                df["time"].values, df["position_error"].values, label=gain
            )
            # set y limits
            ax.set_ylim(0, max(df["position_error"].values) + 0.1)

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position Error (m)")
        ax.legend()

        plt.savefig(f"{directory}/position_error_{metric}.png")

        # Plot #3 - Position over time for different gains

        fig, ax = plt.subplots(2, 2, figsize=(20, 10))

        colors = ["red", "green", "blue"]

        for i, (df, gain) in enumerate(zip(dfs, gains)):
            x, y = i // 2, i % 2
            for pos, color in zip(["x", "y", "z"], colors):
                ax[x, y].plot(
                    df["time"].values,
                    df[pos].values,
                    label=pos,
                    color=color,
                )
                ax[x, y].plot(
                    df["time"].values,
                    df[f"{pos}_"].values,
                    label=f"{pos}_",
                    linestyle="--",
                    color=color,
                )
                ax[x, y].set_ylim(-0.26, 0.36)

            ax[x, y].set_title(f"Gain: {gain}")
            ax[x, y].set_xlabel("Time (s)")
            ax[x, y].set_ylabel("End Effector Position")
            ax[x, y].legend(loc="lower left")

        plt.savefig(f"{directory}/position_{metric}.png")

        # Plot 4 - Joint states over time for different gains

        fig, ax = plt.subplots(2, 2, figsize=(10, 10))

        for i, (df, gain) in enumerate(zip(dfs, gains)):
            x, y = i // 2, i % 2
            for joint in range(5):
                ax[x, y].plot(
                    df["time"].values,
                    df[f"joint_{joint}"].values,
                    label=f"joint_{joint}",
                )

            ax[x, y].set_title(f"Gain: {gain}")
            ax[x, y].set_xlabel("Time (s)")
            ax[x, y].set_ylabel("Joint position")

        plt.savefig(f"{directory}/joint_states_{metric}.png")

    if show:
        plt.show()


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    data_dir = "/home/hgtllys/ros2_ws/data"

    if path is not None:
        print("Reading single experiment folder")
        plot_csv_files(path, show=True)
    else:
        print("Reading all folders in data directory")
        for subdir in os.listdir(data_dir):
            plot_csv_files(os.path.join(data_dir, subdir))


if __name__ == "__main__":
    main()
