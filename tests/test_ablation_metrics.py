"""Regression checks for common actuator limits and ablation metrics."""

import unittest

import numpy as np

from vdm_lab.common.logging import compute_metrics
from vdm_lab.common.types import ControlCommand, Path, StepRecord, VehicleConfig
from vdm_lab.common.vehicle import limit_command


class AblationMetricTests(unittest.TestCase):
    def test_common_steer_rate_limit(self):
        vehicle = VehicleConfig(max_steer_rate=1.0, max_steer=2.0)
        previous = ControlCommand(acceleration=0.0, steer=0.2)
        limited = limit_command(
            ControlCommand(acceleration=99.0, steer=-1.0),
            vehicle,
            previous_command=previous,
            dt=0.1,
        )
        self.assertAlmostEqual(limited.acceleration, vehicle.max_accel)
        self.assertAlmostEqual(limited.steer, 0.1)

    def test_metrics_include_accuracy_time_and_compute_fields(self):
        path = Path(
            x=np.array([0.0, 1.0]),
            y=np.array([0.0, 0.0]),
            yaw=np.array([0.0, 0.0]),
            curvature=np.array([0.0, 0.0]),
            s=np.array([0.0, 1.0]),
            target_speed=np.array([1.0, 0.0]),
        )
        records = [
            StepRecord(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                       0, 1.0, 0.0, 0.0, 0.0, 1.0, 2.0),
            StepRecord(1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0,
                       1, -2.0, 0.0, 0.0, 0.0, 0.0, 4.0),
        ]
        metrics = compute_metrics(path, records)
        self.assertAlmostEqual(metrics["rmse_lateral_error_m"], np.sqrt(2.5))
        self.assertAlmostEqual(metrics["p95_lateral_error_m"], 1.95)
        self.assertAlmostEqual(metrics["completion_time_s"], 1.0)
        self.assertAlmostEqual(metrics["control_compute_mean_ms"], 3.0)
        self.assertAlmostEqual(metrics["max_abs_steer_rate_radps"], 0.2)


if __name__ == "__main__":
    unittest.main()
