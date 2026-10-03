"""Independent numeric tests for warm-up bias models (numpy required)."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

MODULE = Path(__file__).resolve().parents[1]/"tools/analyze_bmi088_temperature.py"
spec = importlib.util.spec_from_file_location("thermal",MODULE)
thermal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(thermal)


class ThermalTests(unittest.TestCase):
    def setUp(self):
        self.bias = np.array([-.04,.025,.02])
        self.scale = np.array([1.006,1.003,.998])
        self.gyro_bias = np.array([-.002,.0003,-.0007])
        self.gyro_k = np.array([-.0002,.0001,-.00005])
        self.accel_k = np.array([.0001,-.0002,.00015])
        self.static = dict(accel_bias_m_s2=self.bias.tolist(),accel_scale=self.scale.tolist())
        self.rows = []
        for axis in range(3):
            for sign in (-1,1):
                gravity = np.zeros(3);gravity[axis] = sign*thermal.G
                hot = gravity/self.scale+self.bias
                for temperature in (25,30,35,39):
                    theta = temperature-40
                    self.rows.append(dict(face=f"{axis}:{sign}",temperature_c=temperature,
                       hot_gyro=self.gyro_bias.tolist(),gyro=(self.gyro_bias+self.gyro_k*theta).tolist(),
                       hot_accel=hot.tolist(),accel=(hot+self.accel_k*theta).tolist()))

    def test_linear_gyro_recovers_temperature_bias(self):
        coefficient = thermal.fit_gyro(self.rows,1)
        np.testing.assert_allclose(coefficient[:,0],self.gyro_k,atol=1e-14)
        full = np.zeros((3,2));full[:,0]=coefficient[:,0]
        corrected = np.array([r["gyro"] for r in self.rows])-self.gyro_bias-thermal.gyro_delta(self.rows,full)
        np.testing.assert_allclose(corrected,0,atol=1e-14)

    def test_gravity_norm_bias_recovers_known_thermal_change(self):
        coefficient = thermal.fit_accel_norm(self.rows,self.static,"bias")
        np.testing.assert_allclose(coefficient[:,0],self.accel_k,atol=1e-12)
        np.testing.assert_array_equal(coefficient[:,1],0)
        corrected = thermal.accel_correct(self.rows,self.static,coefficient)
        np.testing.assert_allclose(np.linalg.norm(corrected,axis=1),thermal.G,atol=1e-11)

    def test_none_has_no_synthetic_thermal_correction(self):
        coefficient = thermal.fit_accel_norm(self.rows,self.static,"none")
        np.testing.assert_array_equal(coefficient,np.zeros((3,2)))

    def test_equal_recording_weights(self):
        rows=self.rows+[self.rows[0]]*8
        weight=thermal.weights(rows)
        for face in {r["face"] for r in rows}:
            self.assertAlmostEqual(sum(w for w,r in zip(weight,rows) if r["face"]==face),1)


if __name__ == "__main__":
    unittest.main()
