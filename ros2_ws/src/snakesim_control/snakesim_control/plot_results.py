import os
import matplotlib.pyplot as plt
import pandas as pd
import argparse


def extract_metadata(file_path):
    file_name = os.path.basename(file_path)
    file_name = file_name.split("_")

    metric = file_name[1].split("=")[1]

    k0 = float(file_name[2].replace(".csv", "").split("=")[1])

    return metric, k0


def main():
    parser = argparse.ArgumentParser(
        description='Process the input csv files and generate the figures.')

    dir_path = os.path.dirname(os.path.realpath(__file__))
    dir_path = os.path.join(dir_path, "results")

    parser.add_argument(
        '--filter_metric', type=str, help='The filter metric to use')

    args = parser.parse_args()

    filter_metric = args.filter_metric

    try:
        file_paths = os.listdir(dir_path)
        file_paths = [
            os.path.join(dir_path, file_path) for file_path in file_paths]
    except Exception as e:
        print(f"Error fetching results: {e}")
        return

    man_values = []
    joint_distance_values = []

    for file_path in file_paths:
        df = pd.read_csv(file_path)

        metric, k0 = extract_metadata(file_path)

        if metric != filter_metric:
            continue

        # Appends to list of values

        man_values.append((k0, df["m"].values.tolist()))
        joint_distance_values.append((k0, df["jd"].values.tolist()))

        # Plot joint positions, velocities, and accelerations

        fig, axs = plt.subplots(2, 1, figsize=(8, 8))

        axs[0].plot(df["t"], df["q0"], label="$q_0$", color="red")
        axs[0].plot(df["t"], df["q1"], label="$q_1$", color="green")
        axs[0].plot(df["t"], df["q2"], label="$q_2$", color="blue")

        axs[1].plot(df["t"], df["qd0"], label="$\\dot{q}_0$", color="red")
        axs[1].plot(df["t"], df["qd1"], label="$\\dot{q}_1$", color="green")
        axs[1].plot(df["t"], df["qd2"], label="$\\dot{q}_2$", color="blue")

        y_labels = ["Posição (rad)", "Velocidade (rad/s)"]

        for i, ax in enumerate(axs):
            ax.set_ylabel(y_labels[i])
            ax.legend(loc="best")
            ax.grid(True)

        metric_name = {
            "manipulability": "Manipulabilidade",
            "jointLimit": "Distância das juntas"
        }[metric]

        axs[0].set_title(f"Função objetivo: {metric_name} - $k_0={k0:.3f}$")
        axs[1].set_xlabel("Tempo (s)")

    # Plot all curves of manipulability

    fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    man_values = sorted(man_values, key=lambda x: x[0])

    line_styles = ['-', '--', '-.', ':']
    colors = ['red', 'green', 'blue', 'purple', 'orange', 'black']

    max_man = 0
    for i, (k0, m) in enumerate(man_values):
        line_style = line_styles[i % len(line_styles)]
        ax.plot(
            df["t"], m, label=f"$k_0={k0:.3f}$",
            color=colors[i % len(colors)], linestyle=line_style
        )
        max_man = max(max_man, max(m))

    ax.set_xlabel("Tempo (s)")
    ax.set_title("Medida de Manipulabilidade")
    ax.legend(loc="lower left")
    ax.grid(True)
    ax.set_ylim([0, max_man*1.1])

    # Plot all curves of joint distance

    fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    joint_distance_values = sorted(joint_distance_values, key=lambda x: x[0])

    min_jd = 0
    for i, (k0, jd) in enumerate(joint_distance_values):
        line_style = line_styles[i % len(line_styles)]
        ax.plot(
            df["t"], jd, label=f"$k_0={k0:.3f}$",
            color=colors[i % len(colors)], linestyle=line_style
        )
        min_jd = min(min_jd, min(jd))

    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel("$\\omega(q)$")
    ax.set_title("Distância para o Limite Mêcanico das Juntas")
    ax.legend(loc="lower left")
    ax.grid(True)
    ax.set_ylim([min_jd*1.1, 0])

    plt.show()


if __name__ == "__main__":
    main()
