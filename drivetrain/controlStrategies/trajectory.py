from wpimath.geometry import Pose2d
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.controlStrategies.holonomicDriveController import (
    HolonomicDriveController,
)
from jormungandr.choreo import ChoreoTrajectoryState
from utils.signalLogging import getNowLogger
from utils.singleton import Singleton


class Trajectory(metaclass=Singleton):
    def __init__(self):
        self._name = "Trajectory"
        self.trajHDC = HolonomicDriveController(self._name)
        self.curTrajCmd = None

        self.curTrajCmdActiveLogger = getNowLogger(f"{self._name}/curTrajCmdActive")
    def setCmd(self, cmd: ChoreoTrajectoryState | None):
        """Send commands to the robot for motion as a part of following a trajectory

        Args:
            cmd (PathPlannerState): PathPlanner trajectory sample for the current time, or None for inactive.
        """
        active = 0
        if cmd is not None:
            active = 1
        self.curTrajCmdActiveLogger.logNow(active)
        self.curTrajCmd = cmd

    def update(self, cmdIn: DrivetrainCommand, curPose: Pose2d) -> DrivetrainCommand:
        if self.curTrajCmd is not None:
            return self.trajHDC.update(self.curTrajCmd, curPose)
        else:
            return cmdIn
