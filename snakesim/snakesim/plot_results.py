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


def compute_score(df):
    df = df.copy()
    df["metric_cumsum"] = df["metric"].abs().cumsum() * 0.032
    error_value = compute_position_error(df.iloc[-1], df.iloc[-1])
    return df["metric_cumsum"].values[-1], error_value


def plot_csv_files(directory, show=False):
    metric = None
    gains = []
    dfs = []

    for filename in os.listdir(directory):
        if filename.endswith(".pdf"):
            return

    for filename in os.listdir(directory):
        data = get_data(directory, filename)
        if data is not None:
            df, gain, metric = data
            gains.append(gain)
            dfs.append(df)

    score_label = [f"k_0={gain}-metric={metric}-perror" for gain in gains]
    scores = [compute_score(df) for df in dfs]

    with open(f"{directory}/scores.txt", "w") as f:
        for label, score in zip(score_label, scores):
            f.write(f"{label}: {score}\n")

    if metric is not None:

        # max_len = max([df.shape[0] for df in dfs])

        # dfs = [df.reindex(range(max_len), method="ffill") for df in dfs]

        # for df in dfs:
        #     df["time"] = 0.032 * df.index

        # Plot #1 - Metric value over time for different gains

        fig, ax = plt.subplots(1, 1, figsize=(10, 5))

        for df, gain in zip(dfs, gains):
            label = "$k_0=" + gain.replace(".0", "") + "$"
            ax.plot(
                df["time"].values[1:],
                abs(df["metric"].values[1:]),
                label=label,
            )

        ax.set_xlabel("time (s)")
        ax.set_ylabel(f"{metric.replace('_', ' ')}")

        ax.legend(
            loc="upper center",
            bbox_to_anchor=(0.5, 1.12),
            fancybox=True,
            shadow=True,
            ncol=5,
        )

        ax.grid()

        plt.savefig(f"{directory}/metric_{metric}.pdf", format="pdf")

        # Plot #2 - Position error over time for different gains

        fig, ax = plt.subplots(1, 1, figsize=(10, 5))

        for df, gain in zip(dfs, gains):
            df["position_error"] = df.apply(
                lambda row: compute_position_error(row, df.iloc[-1]),
                axis=1,
            )
            label = "$k_0=" + gain.replace(".0", "") + "$"
            ax.plot(
                df["time"].values, df["position_error"].values, label=label
            )
            # set y limits
            ax.set_ylim(0, max(df["position_error"].values) + 0.1)

        ax.set_xlabel("time (s)")
        ax.set_ylabel("position error (m)")

        ax.legend(
            loc="upper center",
            bbox_to_anchor=(0.5, 1.12),
            fancybox=True,
            shadow=True,
            ncol=5,
        )

        ax.grid()

        plt.savefig(
            f"{directory}/position_error_{metric}.pdf", format="pdf"
        )

        # Plot #3 - Position over time for different gains

        fig, ax = plt.subplots(2, 2, figsize=(10, 7))

        colors = ["red", "green", "blue"]

        for i, (df, gain) in enumerate(zip(dfs, gains)):
            x, y = i // 2, i % 2
            for pos, color in zip(["x", "y", "z"], colors):
                ax[x, y].plot(
                    df["time"].values,
                    df[pos].values,
                    label=f"${pos}$",
                    color=color,
                )
                ax[x, y].plot(
                    df["time"].values,
                    df[f"{pos}_"].values,
                    label=f"${pos}_f$",
                    linestyle="--",
                    color=color,
                )
                ax[x, y].set_ylim(-0.26, 0.36)

            ax[x, y].set_title(f"$k_0={gain.replace('.0', '')}$")
            ax[x, y].set_ylabel("position (m)")

            if x == 1:
                ax[x, y].set_xlabel("time (s)")
                if y == 1:
                    ax[x, y].legend(
                        loc="upper center",
                        bbox_to_anchor=(1.12, 1.35),
                        fancybox=True,
                        shadow=True,
                        ncol=1,
                    )

            ax[x, y].grid()

        plt.savefig(f"{directory}/position_{metric}.pdf", format="pdf")

        # Plot 4 - Joint states over time for different gains

        fig, ax = plt.subplots(2, 2, figsize=(10, 7))

        for i, (df, gain) in enumerate(zip(dfs, gains)):
            x, y = i // 2, i % 2
            for joint in range(5):
                ax[x, y].plot(
                    df["time"].values,
                    df[f"joint_{joint}"].values,
                    label=f"$q_{joint}$",
                )

            ax[x, y].set_title(f"$k_0={gain.replace('.0', '')}$")
            ax[x, y].set_ylabel("Joint position")

            if x == 1:
                ax[x, y].set_xlabel("time (s)")
                if y == 1:
                    ax[x, y].legend(
                        loc="upper center",
                        bbox_to_anchor=(1.12, 1.35),
                        fancybox=True,
                        shadow=True,
                        ncol=1,
                    )

            ax[x, y].grid()

        plt.savefig(f"{directory}/joint_states_{metric}.pdf", format="pdf")

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
            try:
                plot_csv_files(os.path.join(data_dir, subdir))
            except Exception as e:
                print(f"Error processing {subdir}: {e}")


if __name__ == "__main__":
    main()
