from wpilib import Timer

from positionSchemes.RobotPoserCommon import PoseDirectorCommon
from utils.signalLogging import addLog
from positionSchemes._intel._setup import SetupScheme, ArmConsts, ElevConsts
from humanInterface.operatorInterface import OperatorInterface, ReefLeftOrRight
from humanInterface.driverInterface import DriverInterface


# if you can't find something here, it's probably in the _setup file.

class PutL4V1O(SetupScheme):
    def __init__(self, poseDirectorCommon: PoseDirectorCommon):
        super().__init__(arm=poseDirectorCommon.arm, base=poseDirectorCommon.driveTrain, elev=poseDirectorCommon.elevator)
        self.pdc = poseDirectorCommon
        self.arm = poseDirectorCommon.arm
        self.base = poseDirectorCommon.driveTrain
        self.elev = poseDirectorCommon.elevator
        self.oInt: OperatorInterface = poseDirectorCommon.oInt
        self.dInt: DriverInterface = poseDirectorCommon.dInt
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
        #   elev: (position_in, in/s)

        self.totalRuns = 0
        self.bestTag = 0
        addLog("yvn_current_putL4_o_state", lambda: self.currentState, "")
        # DEFINE THESE
        self.elevPlacePos = 60
        self.armPlacePos = 60

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:
                self.setArmCommand(47, -15)
                self.armPlacePos = 47
                armGoalReached = self.arm.getPosition() < self.armPlacePos
                if armGoalReached or self.oInt.skipNext:
                    self.nextState()
            case 1:
                self.setArmCommand(-15, 0)
            case _:
                pass

        state_max = 1
        self.schemeProg = min((self.currentState+(self.localProg*0.9)) / state_max, 1)
