"""Aggregate saved VDM experiments into a table and comparison figure."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


METRICS = (
    ("mean_lateral_error_m", "Mean lateral error [m]"),
    ("rmse_lateral_error_m", "Lateral-error RMSE [m]"),
    ("completion_time_s", "Completion time [s]"),
    ("control_compute_mean_ms", "Mean control compute [ms]"),
)
SPEED_ORDER = {"low": 0, "medium": 1, "high": 2}


def collect_runs(root):
    runs = []
    for metadata_path in sorted(root.rglob("run_metadata.json")):
        metrics_path = metadata_path.with_name("metrics.json")
        if not metrics_path.exists():
            continue
        with metadata_path.open(encoding="utf-8") as file:
            metadata = json.load(file)
        with metrics_path.open(encoding="utf-8") as file:
            metrics = json.load(file)
        run = {**metadata, **metrics, "output_dir": str(metadata_path.parent)}
        runs.append(run)
    return runs


def _ordered(values, key=None):
    return sorted(set(values), key=key)


def save_summary(root):
    root = Path(root)
    runs = collect_runs(root)
    if not runs:
        raise ValueError(f"在 {root} 内未找到同时含 run_metadata.json 和 metrics.json 的实验。")

    columns = [
        "algorithm", "route", "speed_mode", "vehicle", "control_dt_s",
        "reached_goal", "completion_time_s", "elapsed_time_s",
        "mean_lateral_error_m", "rmse_lateral_error_m", "p95_lateral_error_m",
        "max_lateral_error_m", "finish_error_m", "mean_speed_mps",
        "mean_abs_steer_rate_radps", "max_abs_steer_rate_radps",
        "control_compute_mean_ms", "control_compute_p95_ms",
        "control_compute_max_ms", "output_dir",
    ]
    table_path = root / "ablation_summary.csv"
    with table_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(sorted(
            runs,
            key=lambda run: (
                run.get("algorithm", ""), run.get("route", ""),
                SPEED_ORDER.get(run.get("speed_mode"), 99),
            ),
        ))

    algorithms = _ordered(run["algorithm"] for run in runs)
    routes = _ordered(run["route"] for run in runs)
    speeds = _ordered(
        (run["speed_mode"] for run in runs),
        key=lambda value: SPEED_ORDER.get(value, 99),
    )
    row_keys = [(algorithm, route) for algorithm in algorithms for route in routes]
    lookup = {
        (run["algorithm"], run["route"], run["speed_mode"]): run
        for run in runs
    }

    figure, axes = plt.subplots(2, 2, figsize=(15, max(7, 0.7 * len(row_keys) + 3)))
    for axis, (metric, title) in zip(axes.flat, METRICS):
        data = np.full((len(row_keys), len(speeds)), np.nan)
        for row_index, (algorithm, route) in enumerate(row_keys):
            for column_index, speed in enumerate(speeds):
                value = lookup.get((algorithm, route, speed), {}).get(metric)
                if value is not None:
                    data[row_index, column_index] = float(value)
        image = axis.imshow(data, aspect="auto", cmap="viridis")
        axis.set_title(title)
        axis.set_xticks(range(len(speeds)), speeds)
        axis.set_yticks(range(len(row_keys)), [f"{a} | {r}" for a, r in row_keys])
        for row_index in range(data.shape[0]):
            for column_index in range(data.shape[1]):
                value = data[row_index, column_index]
                axis.text(
                    column_index,
                    row_index,
                    "--" if np.isnan(value) else f"{value:.3g}",
                    ha="center",
                    va="center",
                    color="white" if not np.isnan(value) and value > np.nanmean(data) else "black",
                    fontsize=8,
                )
        figure.colorbar(image, ax=axis, fraction=0.046, pad=0.04)

    figure.suptitle("VDM ablation comparison")
    figure.tight_layout()
    figure_path = root / "ablation_summary.png"
    figure.savefig(figure_path, dpi=180)
    plt.close(figure)
    return table_path, figure_path


def main():
    parser = argparse.ArgumentParser(description="汇总 VDM 批量消融实验")
    parser.add_argument("output_root", type=Path, help="包含各实验输出目录的根目录")
    args = parser.parse_args()
    table_path, figure_path = save_summary(args.output_root)
    print(f"summary_csv={table_path}")
    print(f"summary_figure={figure_path}")


if __name__ == "__main__":
    main()
