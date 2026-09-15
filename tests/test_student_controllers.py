"""学生 PP/MPC 的接口、模型与约束回归检查。"""

import math
import unittest

import numpy as np

from vdm_lab.common.reference import ReferenceTracker
from vdm_lab.common.types import ControlCommand, LabConfig, Path, VehicleState
from vdm_lab.common.vehicle import update_state
from vdm_lab.student import mpc, pure_pursuit


class StudentControllerTests(unittest.TestCase):
    def setUp(self):
        self.config = LabConfig()
        s = np.linspace(0.0, 40.0, 81)
        self.path = Path(s, np.zeros_like(s), np.zeros_like(s),
                         np.zeros_like(s), s, np.full_like(s, 3.0))

    def test_linearization_matches_vehicle_and_finite_differences(self):
        cfg = self.config
        for v, yaw, steer in [(0.0, 0.0, 0.0), (4.0, 3.12, 0.25), (7.0, -2.1, -0.4)]:
            z = np.array([1200.0, -500.0, v, yaw])
            u = np.array([0.3, steer])
            a, b, c = mpc.linear_model(v, yaw, steer, cfg)
            predicted = mpc._model_step(z, *u, cfg)
            np.testing.assert_allclose(a @ z + b @ u + c, predicted, atol=1e-10)
            plant, _ = update_state(VehicleState(z[0], z[1], yaw, v), ControlCommand(*u), cfg.vehicle, cfg.sim.dt)
            np.testing.assert_allclose(predicted[:3], [plant.x, plant.y, plant.v], atol=1e-10)
            self.assertAlmostEqual(math.sin(predicted[3] - plant.yaw), 0.0)
            eps = 1e-5
            numeric_a = np.column_stack([
                (mpc._model_step(z + eps * e, *u, cfg) - mpc._model_step(z - eps * e, *u, cfg)) / (2 * eps)
                for e in np.eye(4)
            ])
            numeric_b = np.column_stack([
                (mpc._model_step(z, *(u + eps * e), cfg) - mpc._model_step(z, *(u - eps * e), cfg)) / (2 * eps)
                for e in np.eye(2)
            ])
            np.testing.assert_allclose(a, numeric_a, atol=1e-7)
            np.testing.assert_allclose(b, numeric_b, atol=1e-7)

    def test_reference_yaw_wrap_and_endpoint(self):
        self.path.yaw = np.arctan2(np.sin(np.linspace(3.0, 3.5, 81)), np.cos(np.linspace(3.0, 3.5, 81)))
        state = VehicleState(12.0, 0.0, -3.13, 3.0)
        ref = ReferenceTracker(self.path).nearest(state)
        z_ref = mpc.nearest_horizon_reference(state, ref, self.config)
        self.assertEqual(z_ref.shape, (4, self.config.controller.mpc_horizon + 1))
        self.assertLess(abs(z_ref[3, 0] - state.yaw), 0.2)
        self.assertLess(np.max(np.abs(np.diff(z_ref[3]))), 0.1)
        ref.nearest_index = len(self.path.s) - 1
        z_ref = mpc.nearest_horizon_reference(state, ref, self.config)
        np.testing.assert_allclose(z_ref[0], self.path.x[-1])
        np.testing.assert_allclose(z_ref[2], 0.0)

    def test_pp_turn_direction_rate_and_endpoint(self):
        state = VehicleState(0.0, 1.0, 0.0, 3.0)
        ref = ReferenceTracker(self.path).nearest(state)
        command = pure_pursuit.control(state, ref, ControlCommand(0.0, 0.0), self.config)
        self.assertLess(command.steer, 0.0)
        self.assertLessEqual(abs(command.steer), self.config.vehicle.max_steer_rate * self.config.sim.dt)
        ref.nearest_index = len(self.path.s) - 1
        command = pure_pursuit.control(VehicleState(40.0, 0.0, 0.0, 0.0), ref, command, self.config)
        self.assertTrue(np.isfinite(command.steer))
        self.assertEqual(command.acceleration, 0.0)

    def test_mpc_startup_prediction_and_constraints(self):
        cfg = self.config
        state = VehicleState(0.0, 1.0, 0.0, 0.0)
        ref = ReferenceTracker(self.path).nearest(state)
        previous = ControlCommand(0.0, 0.1)
        command = mpc.control(state, ref, previous, cfg)
        self.assertGreater(command.acceleration, 0.0)
        self.assertEqual(command.prediction.shape, (4, cfg.controller.mpc_horizon + 1))
        z0 = np.array([state.x, state.y, state.v, state.yaw])
        acceleration, steer, prediction = mpc.solve_linear_mpc(
            mpc.nearest_horizon_reference(state, ref, cfg), command.prediction, z0, previous.steer, cfg,
        )
        tolerance = 1e-4
        self.assertTrue(np.all(acceleration <= cfg.vehicle.max_accel + tolerance))
        self.assertTrue(np.all(acceleration >= -cfg.vehicle.max_decel - tolerance))
        self.assertTrue(np.all(np.abs(steer) <= cfg.vehicle.max_steer + tolerance))
        self.assertTrue(np.all(np.abs(np.diff(np.r_[previous.steer, steer])) <= cfg.vehicle.max_steer_rate * cfg.sim.dt + tolerance))
        self.assertTrue(np.all(prediction[2] >= -tolerance))
        self.assertTrue(np.all(prediction[2] <= cfg.vehicle.max_speed + tolerance))
        np.testing.assert_allclose(prediction[:, 0], z0, atol=tolerance)


if __name__ == "__main__":
    unittest.main()
