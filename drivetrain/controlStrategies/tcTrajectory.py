from wpimath.geometry import Pose2d
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.controlStrategies.holonomicDriveController import (
    HolonomicDriveController,
)
from jormungandr.choreo import ChoreoTrajectoryState
from utils.signalLogging import getNowLogger
from utils.singleton import Singleton


class TCTrajectory(metaclass=Singleton):
    def __init__(self):
        self._name = "TCTrajectory"
        self.trajHDC = HolonomicDriveController(self._name)
        self.curTrajCmdFromChoreaAuton = None
        self.curTrajCmdFromPoser = None

        self.curTrajCmdActiveLogger = getNowLogger(f"{self._name}/curTrajCmdActive")

    def _activeLogger(self, cmd: ChoreoTrajectoryState | None):
        active = 0
        if cmd is not None:
            active = 1
        self.curTrajCmdActiveLogger.logNow(active)

    def setCmdFromChoreoAuton(self, cmd: ChoreoTrajectoryState | None):
        """Send commands to the robot for motion as a part of following a trajectory from auton

        Args:
            cmd (PathPlannerState): PathPlanner trajectory sample for the current time, or None for inactive.
        """
        self._activeLogger(cmd)
        self.curTrajCmdFromChoreaAuton = cmd

    def setCmdFromPoser(self, cmd: ChoreoTrajectoryState | None):
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
