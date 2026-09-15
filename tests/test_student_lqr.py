import unittest

import numpy as np

from vdm_lab.common.types import LabConfig
from vdm_lab.student import lqr_dynamic, lqr_kinematic


class StudentLQRTests(unittest.TestCase):
    def test_riccati_scalar_known_solution(self):
        # A=B=Q=R=1 的稳定反馈增益为黄金分割比的倒数。
        one = np.ones((1, 1))
        for module in (lqr_kinematic, lqr_dynamic):
            gain = module.solve_lqr(one, one, one, one, 1e-12, 1000)
            self.assertAlmostEqual(gain[0, 0], (np.sqrt(5.0) - 1.0) / 2.0)

    def test_models_and_feedback_are_finite_and_stable(self):
        cfg = LabConfig()
        for module, builder in (
            (lqr_kinematic, lqr_kinematic.build_kinematic_model),
            (lqr_dynamic, lqr_dynamic.build_dynamic_model),
        ):
            for speed in (0.5, 3.0, 7.0):
                a, b = builder(speed, cfg)
                gain = module.solve_lqr(a, b, cfg.controller.lqr_q, cfg.controller.lqr_r, 1e-9, 5000)
                self.assertEqual(a.shape, (4, 4))
                self.assertEqual(b.shape, (4, 1))
                self.assertTrue(np.all(np.isfinite(gain)))
                self.assertLess(np.max(np.abs(np.linalg.eigvals(a - b @ gain))), 1.0)

    def test_dynamic_zero_speed_protection_and_feedforward_symmetry(self):
        cfg = LabConfig()
        a, b = lqr_dynamic.build_dynamic_model(0.0, cfg)
        guarded_a, guarded_b = lqr_dynamic.build_dynamic_model(cfg.controller.lqr_min_model_speed, cfg)
        np.testing.assert_allclose(a, guarded_a)
        np.testing.assert_allclose(b, guarded_b)
        gain = lqr_dynamic.solve_lqr(a, b, cfg.controller.lqr_q, cfg.controller.lqr_r, 1e-8, 5000)
        feedforward = lqr_dynamic.dynamic_feedforward
        self.assertEqual(feedforward(0.0, 0.0, gain, cfg), 0.0)
        self.assertAlmostEqual(feedforward(3.0, 0.1, gain, cfg), -feedforward(3.0, -0.1, gain, cfg))


if __name__ == '__main__':
    unittest.main()
