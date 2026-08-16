import unittest

from web_controller import RobotBackend


class ControlScalingTests(unittest.TestCase):
    def test_chassis_and_lift_use_independent_scales(self) -> None:
        backend = RobotBackend()
        backend.config["speed_scale"] = 0.20
        backend.config["lift_speed_scale"] = 0.60
        backend.motion = (1.0, -1.0, 0.5, 1.0)

        self.assertEqual(backend._scaled_motion(), (200, -200, 100, 600))

    def test_motion_and_scales_are_clamped(self) -> None:
        backend = RobotBackend()
        backend.config["speed_scale"] = 0.01
        backend.config["lift_speed_scale"] = 2.0
        backend.motion = (2.0, -2.0, 0.0, -2.0)

        self.assertEqual(backend._scaled_motion(), (50, -50, 0, -1000))


if __name__ == "__main__":
    unittest.main()
