import math

from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._setup import SetupScheme, ArmConsts, ElevConsts
from positionSchemes._posintelligence import PlacementIntelligence
from positionSchemes.RobotPoserCommon import PoseDirectorCommon
from humanInterface.operatorInterface import ReefLeftOrRight

from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS
from wpimath.geometry import Pose2d

# if you can't find something here, it's probably in the _setup file.

class PutL3V2O(SetupScheme):
    def __init__(self, poseDirectorCommon: PoseDirectorCommon):
        super().__init__(arm=poseDirectorCommon.arm, base=poseDirectorCommon.driveTrain, elev=poseDirectorCommon.elevator)
        self.pdc = poseDirectorCommon
        self.arm = poseDirectorCommon.arm
        self.base = poseDirectorCommon.driveTrain
        self.elev = poseDirectorCommon.elevator
        self.oInt = poseDirectorCommon.oInt
        self.dInt = poseDirectorCommon.dInt

        self.pdReefSideState = self.dInt.dPadState
        self.pdSideOfReef = -1
        if self.pdReefSideState == ReefLeftOrRight.RIGHT:
            self.pdSideOfReef = 1
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

        # structure:
        #   base: (Pose2d, velx, vely, velt)
        #   arm: (position_deg, deg/s)
        #   elev: (pasition_in, in/s)

        self.totalRuns = 0
        self.bestTag = 0
        addLog("yvn_current_placeL3_state", lambda: self.currentState, "")
        addLog("yvn_placeL3_runs", lambda: self.totalRuns, "")
        self.placementIntel = PlacementIntelligence(self.base)
        # DEFINE THESE
        self.elevPlacePos = 34
        self.elevDistanceDownNeeded = 6
        self.armPlacePos = 55

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:
                self.elevCmd = (self.elevPlacePos-self.elevDistanceDownNeeded, 0)
                self.nextState()
            case 1:
                elevGoal = self.elevPlacePos-self.elevDistanceDownNeeded
                elevGoalReached = math.isclose(self.elev.getPosition(), elevGoal, abs_tol=1)
                if elevGoalReached:
                    self.nextState()
            case 2:
                pass
            case _:
                pass

        state_max = 2
        # when calculating the scheme prog, we can also add in local progress to show something as we go thru state.
        self.schemeProg = min((self.currentState+self.localProg) / (state_max), 1)
