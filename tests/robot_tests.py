"""
Proof of concept to test robots

This POC was made by deconstructing pytest_plugin.py so that it is no longer a plugin but a class that provides
fixtures.

To run / debug this:

pytest tests/robot_tests.py --no-header -vvv -s

"""

import pytest

from wpilib.timedrobotpy import TimedRobotPy
from robot import MyRobot


def run_practice(control: "TestController"):
    """Runs through the entire span of a practice match"""

    with control.run_robot():
        # Run disabled for a short period
        control.step_timing(seconds=0.5, autonomous=True, enabled=False)

        # Run autonomous + enabled for 15 seconds
        control.step_timing(seconds=15, autonomous=True, enabled=True)

        # Disabled for another short period
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)

        # Run teleop + enabled for 2 minutes
        control.step_timing(seconds=120, autonomous=False, enabled=True)

class TestThings():


    @classmethod
    @pytest.fixture(scope="class", autouse=True)
    def myrobot_class(cls) -> type[MyRobot]:
        return MyRobot

    @classmethod
    @pytest.fixture(scope="class", autouse=True)
    def robots_sim_enable_physics(cls) -> bool:
        return False


    def test_iterative(self, getTestController, robot_with_sim_setup_teardown):
        """Ensure that all states of the iterative robot run"""
        run_practice(getTestController)
        assert robot_with_sim_setup_teardown.count > 0

    def test_iterative_again(self, getTestController, robot_with_sim_setup_teardown):
        """Ensure that all states of the iterative robot run"""
        run_practice(getTestController)
        assert robot_with_sim_setup_teardown.count > 0
