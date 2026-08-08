import json
import unittest
from pathlib import Path

from velocity_servo_tag.lens_model import ZoomLensModel


CALIBRATION = (
    Path(__file__).parents[1]
    / "config"
    / "lens_calibration"
    / "camera0_zoom_v2.json"
)


class LensModelV2Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ZoomLensModel.from_json(CALIBRATION)
        cls.raw = json.loads(CALIBRATION.read_text(encoding="utf-8"))

    def test_control_anchors_are_preserved(self):
        for row, distance in enumerate(self.model.distances):
            for column, zoom in enumerate(self.model.zooms):
                prediction = self.model.predict_control(zoom, distance)
                self.assertAlmostEqual(
                    prediction.focus_steps,
                    self.raw["control_grids"]["focus_steps"][row][column],
                )
                self.assertAlmostEqual(
                    prediction.fx,
                    self.raw["control_grids"]["fx_px"][row][column],
                )
                self.assertAlmostEqual(
                    prediction.fy,
                    self.raw["control_grids"]["fy_px"][row][column],
                )

    def test_inverse_returns_anchor_zoom(self):
        distance = 0.8
        for zoom in self.model.zooms:
            value = self.model.predict_control(zoom, distance)
            solution = self.model.solve_fx_fy(value.fx, value.fy, distance, 1.0)
            self.assertAlmostEqual(solution.zoom_steps, zoom, places=6)

    def test_camera_info_keeps_k_and_d_anchor_pair(self):
        camera = self.model.predict_camera(2400, 1.0)
        self.assertAlmostEqual(camera.fx, 5552.49339803842)
        self.assertAlmostEqual(camera.cx, 898.1844700478205)
        self.assertAlmostEqual(camera.distortion[0], 0.09806306481419307)
        self.assertAlmostEqual(camera.distortion[4], 59.21509720118577)

    def test_interpolation_is_finite_inside_domain(self):
        control = self.model.predict_control(1000, 0.65)
        camera = self.model.predict_camera(1000, 0.65)
        self.assertGreater(control.fx, 2690.0)
        self.assertLess(control.fx, 3150.0)
        self.assertGreater(camera.fx, 2690.0)
        self.assertEqual(len(camera.distortion), 5)

    def test_out_of_domain_is_rejected(self):
        with self.assertRaises(ValueError):
            self.model.predict_control(1200, 1.2)
        with self.assertRaises(ValueError):
            self.model.predict_camera(2500, 0.8)

    def test_inconsistent_fx_fy_is_rejected(self):
        with self.assertRaises(ValueError):
            self.model.solve_fx_fy(2300.0, 4500.0, 0.8, 80.0)


if __name__ == "__main__":
    unittest.main()
