"""Test suite for Robotics VLA Torque Sentinel solution."""
import unittest
from robotics_vla_torque_sentinel import RoboticsVLATorqueSentinel

class TestRoboticsVLATorqueSentinel(unittest.TestCase):

    def test_torque_evaluation(self):
        sentinel = RoboticsVLATorqueSentinel(joint_count=28, max_torque_nm=350.0)
        test_torques = [200.0] * 27 + [450.0]  # 1 anomalous joint
        res = sentinel.evaluate_torque_command(target_torques_nm=test_torques, com_velocity_ms=1.2)
        
        self.assertEqual(res["anomalies_clamped"], 1)
        self.assertEqual(res["safety_status"], "ROBOTICS_TORQUE_NOMINAL")

if __name__ == "__main__":
    unittest.main()
