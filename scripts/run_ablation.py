"""Run the PP/LQR/MPC route-speed ablation with stable output directories."""

import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path


DEFAULT_ALGORITHMS = ("pp", "lqr_kinematic", "mpc")
DEFAULT_ROUTES = ("double_lane_change", "right_angle", "s_curve")
DEFAULT_SPEED_MODES = ("low", "medium", "high")


def _output_root(value):
    if value is not None:
        return Path(value)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path("outputs") / f"ablation_{stamp}"


def main():
    parser = argparse.ArgumentParser(description="批量运行 VDM 路径和速度消融实验")
    parser.add_argument("--algorithms", nargs="+", default=DEFAULT_ALGORITHMS)
    parser.add_argument("--routes", nargs="+", default=DEFAULT_ROUTES)
    parser.add_argument("--speed-modes", nargs="+", default=DEFAULT_SPEED_MODES)
    parser.add_argument("--vehicle", default="student_car")
    parser.add_argument("--version", choices=["solution", "student"], default="solution")
    parser.add_argument("--dt", type=float, default=None)
    parser.add_argument("--max-time", type=float, default=None)
    parser.add_argument("--output-root", type=Path, default=None)
    parser.add_argument("--save-gif", action="store_true", help="为每组额外保存 GIF，耗时较长")
    parser.add_argument("--keep-going", action="store_true", help="某组失败后继续后续实验")
    args = parser.parse_args()

    if args.dt is not None and args.dt <= 0.0:
        raise SystemExit("--dt 必须为正数。")
    output_root = _output_root(args.output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise SystemExit(f"输出根目录已存在且非空：{output_root}")
    output_root.mkdir(parents=True, exist_ok=True)

    repository_root = Path(__file__).resolve().parents[1]
    experiment_entry = repository_root / "run_experiment.py"
    failures = []
    total = len(args.algorithms) * len(args.routes) * len(args.speed_modes)
    current = 0
    for algorithm in args.algorithms:
        for route in args.routes:
            for speed_mode in args.speed_modes:
                current += 1
                output_dir = output_root / f"{algorithm}__{route}__{speed_mode}"
                command = [
                    sys.executable, str(experiment_entry),
                    "--algo", algorithm,
                    "--version", args.version,
                    "--route", route,
                    "--speed-mode", speed_mode,
                    "--vehicle", args.vehicle,
                    "--output-dir", str(output_dir),
                    "--save-log", "--save-fig",
                ]
                if args.dt is not None:
                    command.extend(["--dt", str(args.dt)])
                if args.max_time is not None:
                    command.extend(["--max-time", str(args.max_time)])
                if args.save_gif:
                    command.append("--save-gif")
                print(f"[{current}/{total}] {' '.join(command)}", flush=True)
                completed = subprocess.run(command, cwd=repository_root)
                if completed.returncode:
                    failures.append((algorithm, route, speed_mode, completed.returncode))
                    if not args.keep_going:
                        raise SystemExit(f"实验失败：{failures[-1]}")

    summarize = repository_root / "scripts" / "summarize_ablation.py"
    subprocess.run([sys.executable, str(summarize), str(output_root)], cwd=repository_root, check=True)
    print(f"ablation_root={output_root}")
    if failures:
        print(f"failed_runs={failures}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()
