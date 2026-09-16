import csv
import json
from dataclasses import asdict
from datetime import datetime
from pathlib import Path as FsPath

import numpy as np


def create_output_dir(label, root="outputs"):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = FsPath(root) / f"{stamp}_{label}"
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def save_records(output_dir, records):
    csv_path = output_dir / "trajectory.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(asdict(records[0]).keys()))
        writer.writeheader()
        for record in records:
            writer.writerow(asdict(record))
    return csv_path


def save_reference_path(output_dir, path):
    """
    Save the processed reference path used by the controllers.

    For GPX routes, geographic latitude/longitude are preserved alongside
    local x/y, making it easy to compare simulation and real GNSS logs later.
    """
    csv_path = output_dir / "reference_path.csv"

    lat = getattr(path, "lat", None)
    lon = getattr(path, "lon", None)
    elevation = getattr(path, "elevation", None)

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "index",
                "s_m",
                "x_m",
                "y_m",
                "yaw_rad",
                "curvature_1pm",
                "target_speed_mps",
                "lat_deg",
                "lon_deg",
                "elevation_m",
            ]
        )
        for i in range(len(path.x)):
            writer.writerow(
                [
                    i,
                    path.s[i],
                    path.x[i],
                    path.y[i],
                    path.yaw[i],
                    path.curvature[i],
                    path.target_speed[i],
                    "" if lat is None else lat[i],
                    "" if lon is None else lon[i],
                    "" if elevation is None else elevation[i],
                ]
            )
    return csv_path


def save_predictions(output_dir, predictions):
    if not predictions:
        return None
    path = output_dir / "mpc_predictions.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "horizon_index", "x", "y", "v", "yaw"])
        for time, prediction in predictions:
            for i in range(prediction.shape[1]):
                writer.writerow(
                    [
                        time,
                        i,
                        prediction[0, i],
                        prediction[1, i],
                        prediction[2, i],
                        prediction[3, i],
                    ]
                )
    return path


def _safe_nanmax(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0 or np.all(np.isnan(values)):
        return float("nan")
    return float(np.nanmax(values))


def _safe_nanmean(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0 or np.all(np.isnan(values)):
        return float("nan")
    return float(np.nanmean(values))


def compute_metrics(path, records):
    if not records:
        raise ValueError("没有仿真记录，无法计算指标。")

    signed_lateral_errors = np.array(
        [r.lateral_error for r in records], dtype=float
    )
    lateral_errors = np.abs(signed_lateral_errors)
    heading_errors = np.array([abs(r.heading_error) for r in records], dtype=float)
    speeds = np.array([r.speed for r in records], dtype=float)
    steers = np.array([abs(r.steer) for r in records], dtype=float)
    accelerations = np.array([abs(r.acceleration) for r in records], dtype=float)
    normal_accels = np.array([abs(r.normal_accel) for r in records], dtype=float)
    betas = np.array([abs(r.beta) for r in records], dtype=float)
    yaw_rates = np.array([abs(r.yaw_rate) for r in records], dtype=float)
    compute_times = np.array([r.control_compute_ms for r in records], dtype=float)
    target_speeds = np.array([r.target_speed for r in records], dtype=float)
    times = np.array([r.time for r in records], dtype=float)
    if len(records) > 1:
        dt = np.diff(times)
        steer_rates = np.diff(np.array([r.steer for r in records], dtype=float)) / dt
        steer_rate_abs = np.abs(steer_rates[np.isfinite(steer_rates)])
    else:
        steer_rate_abs = np.empty(0, dtype=float)
    last = records[-1]
    finish_error = float(np.hypot(last.x - path.x[-1], last.y - path.y[-1]))
    reached_goal = bool(finish_error < 1.5 and last.speed < 0.5)
    elapsed_time = float(times[-1] - times[0]) if len(times) > 1 else 0.0

    return {
        "mean_lateral_error_m": float(lateral_errors.mean()),
        "max_lateral_error_m": float(lateral_errors.max()),
        "rmse_lateral_error_m": float(np.sqrt(np.mean(signed_lateral_errors ** 2))),
        "p95_lateral_error_m": float(np.percentile(lateral_errors, 95)),
        "finish_error_m": finish_error,
        "mean_heading_error_rad": float(heading_errors.mean()),
        "max_steer_rad": float(steers.max()),
        "max_acceleration_mps2": float(accelerations.max()),
        "max_normal_acceleration_mps2": float(normal_accels.max()),
        "max_side_slip_beta_rad": _safe_nanmax(betas),
        "max_yaw_rate_radps": _safe_nanmax(yaw_rates),
        "min_speed_mps": float(speeds.min()),
        "mean_speed_mps": float(speeds.mean()),
        "mean_target_speed_mps": float(target_speeds.mean()),
        "max_speed_mps": float(speeds.max()),
        "mean_abs_steer_rate_radps": _safe_nanmean(steer_rate_abs),
        "max_abs_steer_rate_radps": _safe_nanmax(steer_rate_abs),
        "control_compute_mean_ms": _safe_nanmean(compute_times),
        "control_compute_p95_ms": float(np.percentile(compute_times, 95)),
        "control_compute_max_ms": _safe_nanmax(compute_times),
        "elapsed_time_s": elapsed_time,
        "completion_time_s": elapsed_time if reached_goal else None,
        "steps": len(records),
        "reached_goal": reached_goal,
    }


def save_metrics(output_dir, metrics):
    path = output_dir / "metrics.json"
    with path.open("w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)
    return path


def save_run_metadata(output_dir, metadata):
    path = FsPath(output_dir) / "run_metadata.json"
    with path.open("w", encoding="utf-8") as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)
    return path
