from wpilib import Timer
from wpimath.geometry import Pose2d
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.controlStrategies.holonomicDriveController import (
    HolonomicDriveController,
)
from choreo.trajectory import SwerveSample
from utils.signalLogging import getNowLogger


class TrajectoryGuts:
    def __init__(self):
        self._name = "TrajectoryGuts"
        self.trajHDC = HolonomicDriveController(self._name)
        self.curTrajCmdFromChoreaAuton = None
        self.curTrajCmdFromPoser = None

        self.curTrajCmdActiveLogger = getNowLogger(f"{self._name}/curTrajCmdActive")

    def setName(self, name):
        self._name = name

    def _activeLogger(self, cmd: SwerveSample | None):
        active = 0
        if cmd is not None:
            active = 1
        self.curTrajCmdActiveLogger.logNow(active)

    def setCmdFromChoreoAuton(self, cmd: SwerveSample | None):
        """Send commands to the robot for motion as a part of following a trajectory from auton

        Args:
            cmd (PathPlannerState): PathPlanner trajectory sample for the current time, or None for inactive.
        """
        self._activeLogger(cmd)
        self.curTrajCmdFromChoreaAuton = cmd
        print(f"setCmdFromChoreoAuton time = {Timer.getFPGATimestamp():.3f}s cmd={cmd}")

    def setCmdFromPoser(self, cmd: SwerveSample | None):
        """Send commands to the robot for motion as a part of following a trajectory from poser

        Args:
            cmd (PathPlannerState): PathPlanner trajectory sample for the current time, or None for inactive.
        """
        self._activeLogger(cmd)
        self.curTrajCmdFromPoser = cmd

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:
        if self.curTrajCmdFromPoser is not None:
            return self.trajHDC.update(self.curTrajCmdFromPoser, curPose)
        if self.curTrajCmdFromChoreaAuton is not None:
            return self.trajHDC.update(self.curTrajCmdFromChoreaAuton, curPose)
        else:
            return cmdIn
