"""Numerical fit/selection regression, separate from firmware host tests."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

MODULE = Path(__file__).resolve().parents[1] / "tools/analyze_bmi088_calibration.py"
spec = importlib.util.spec_from_file_location("calibration", MODULE)
cal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cal)


class CalibrationTests(unittest.TestCase):
    def test_tilted_six_faces_recover_known_bias_scale(self):
        bias = np.array([-.04, .025, .02])
        scale = np.array([1.006, 1.003, .998])
        vectors = []
        for axis in range(3):
            for sign in (-1, 1):
                v = np.array([.02, -.015, .01])
                v[axis] = sign
                v = v / np.linalg.norm(v) * cal.G
                vectors.append(v / scale + bias)
        fitted_bias, fitted_scale = cal.fit_accel(vectors)
        np.testing.assert_allclose(fitted_bias, bias, atol=1e-11)
        np.testing.assert_allclose(fitted_scale, scale, atol=1e-11)
        # Gravity is retained and no cross-axis rotation is introduced.
        corrected = (vectors - fitted_bias) * fitted_scale
        np.testing.assert_allclose(np.linalg.norm(corrected, axis=1), cal.G, atol=1e-11)

    def test_missing_or_duplicate_face_rejected(self):
        vectors = np.concatenate([np.eye(3), -np.eye(3)]) * cal.G
        vectors[-1] = vectors[0]
        with self.assertRaises(ValueError):
            cal.fit_accel(vectors)

    def test_warmup_movement_guard_and_full_bin_selection(self):
        bins = [dict(second=i, n=500, temperature_min=40 if i >= 90 else 30,
                     temperature_max=40 if i >= 90 else 30,
                     gyro_peak=.8, gyro_std=[.2,.28,.24], accel_std=[.03,.03,.03])
                for i in range(480)]
        bins[205]["gyro_peak"] = 50
        bins[300]["n"] = 100
        bins[350]["temperature_max"] = 41
        ready, movement, accepted = cal.selection({"seconds": bins})
        self.assertEqual(ready, 180)
        self.assertEqual(movement, [205])
        self.assertTrue(all(x >= 180 for x in accepted))
        self.assertFalse(set(range(203,208)) & set(accepted))
        self.assertNotIn(300, accepted)
        self.assertNotIn(350, accepted)
        self.assertIn(208, accepted)

    def test_no_stable_temperature_rejected(self):
        with self.assertRaises(ValueError):
            cal.selection({"seconds": []})


if __name__ == "__main__":
    unittest.main()
