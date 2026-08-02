import unittest

from control_model import ControllerModel


class ControllerModelTests(unittest.TestCase):
    def test_zero_error_is_neutral(self):
        controller = ControllerModel()
        result = controller.step(controller.setpoint, 0.01)
        self.assertEqual(result.command, 0.0)

    def test_command_saturates(self):
        controller = ControllerModel()
        result = controller.step(10.0, 0.01)
        self.assertEqual(result.command, -255.0)

    def test_integral_is_bounded(self):
        controller = ControllerModel()
        for _ in range(5000):
            controller.previous_angle = 3.0
            controller.step(3.0, 0.01)
        self.assertLessEqual(abs(controller.integral), 100.0)

    def test_invalid_timing_faults(self):
        controller = ControllerModel()
        result = controller.step(2.0, 0.0)
        self.assertEqual(result.state, "FAULT")
        self.assertEqual(result.command, 0.0)

    def test_angle_limit_faults_and_latches(self):
        controller = ControllerModel()
        result = controller.step(18.0, 0.01)
        self.assertEqual(result.state, "FAULT")
        self.assertEqual(controller.step(2.0, 0.01).command, 0.0)


if __name__ == "__main__":
    unittest.main()
