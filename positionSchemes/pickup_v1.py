import math

from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._setup import SetupScheme, ArmConsts, ElevConsts
from positionSchemes.RobotPoserCommon import PoseDirectorCommon
from positionSchemes._posintelligence import PickupIntelligence

#if you can't find something here, it's probably in the _setup file.

class PickupV1(SetupScheme):
    def __init__(self, poseDirectorCommon: PoseDirectorCommon):
        super().__init__(arm=poseDirectorCommon.arm, base=poseDirectorCommon.driveTrain, elev=poseDirectorCommon.elevator)
        self.pdc = poseDirectorCommon
        self.arm = poseDirectorCommon.arm
        self.base = poseDirectorCommon.driveTrain
        self.elev = poseDirectorCommon.elevator
        self.oInt = poseDirectorCommon.oInt

        self.armConst = ArmConsts()
        self.elevConst = ElevConsts()
        self.currentState = 0

        self.startTime = Timer.getFPGATimestamp()
        self.changeInTime = 0
        self.waitTimes = {}
        self.schemeProg = 0
        self.setDriveTrainBaseCommand(None)
        self.armCmd = None
        self.elevCmd = None

        self.totalRuns = 0
        self.bestTag = 0
        addLog("yvn_current_pickup_state", lambda: self.currentState, "")
        addLog("yvn_pickup_runs", lambda: self.totalRuns, "")

        self.elevPickupPose = 45.30  # 47.6875  #inches

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:  # initializing
                self.bestTag = PickupIntelligence(self.base).decidePickupPose(self.inchesToMeters(10))
                self.setDriveTrainBaseCommand(self.bestTag)
                self.armCmd = None
                self.elevCmd = None
                if self.completedAwait("awaitbasecmdsend", 0.2):
                    self.nextState()
            case 1:
                self.elevCmd = (self.elevPickupPose, 0)
                if self.completedTrajectory(self.base):
                    self.nextState()
            case 2:
                elevGoalReached = math.isclose(self.elev.getPosition(), self.elevPickupPose, abs_tol=0.5)
                if elevGoalReached:
                    self.nextState()
            case 3:
                self.bestTag = PickupIntelligence(self.base).decidePickupPose(self.inchesToMeters(1))
                self.setDriveTrainBaseCommand(self.bestTag)
                pass
            case _:
                pass

        state_max = 3
        # when calculating the scheme prog, we can also add in local progress to show something as we go thru state.
        self.schemeProg = min((self.currentState) / (state_max), 1)
