import unittest

from velocity_servo_tag.lens_model import ZoomIntrinsicsModel


Z = [0, 400, 800, 1200, 1600, 2000, 2400]
FX = [2035.797529, 2331.545143, 2713.737027, 3137.392785, 3684.865723, 4505.996199, 5494.414672]
FY = [2034.742218, 2331.622073, 2714.807451, 3138.780385, 3683.854745, 4503.318979, 5490.231460]
CX = [946.313693, 931.606961, 945.418505, 930.987463, 930.964585, 928.012695, 894.846174]
CY = [749.155224, 766.482076, 774.908282, 766.529524, 757.853146, 756.246991, 746.485232]


class LensModelTest(unittest.TestCase):
    def setUp(self):
        self.model = ZoomIntrinsicsModel(Z, FX, FY, CX, CY, [0.0] * 35)

    def test_exact_calibration_pair_returns_exact_zoom(self):
        for zoom, fx, fy in zip(Z, FX, FY):
            solution = self.model.solve_fx_fy(fx, fy, 20.0)
            self.assertAlmostEqual(solution.zoom_steps, zoom, places=6)

    def test_inconsistent_pair_is_rejected(self):
        with self.assertRaises(ValueError):
            self.model.solve_fx_fy(3000.0, 4000.0, 20.0)

    def test_out_of_range_is_rejected(self):
        with self.assertRaises(ValueError):
            self.model.solve_fx_fy(1000.0, 1000.0, 20.0)

    def test_interpolation_remains_between_neighbors(self):
        state = self.model.intrinsics(1000.0)
        self.assertGreater(state.fx, FX[2])
        self.assertLess(state.fx, FX[3])
        self.assertGreater(state.fy, FY[2])
        self.assertLess(state.fy, FY[3])


if __name__ == "__main__":
    unittest.main()
