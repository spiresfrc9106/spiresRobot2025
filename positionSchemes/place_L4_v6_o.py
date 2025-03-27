import math
from Elevatorandmech.armtest import ArmControl
from Elevatorandmech.elevatortest import ElevatorControl
from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._setup import SetupScheme, ArmConsts, ElevConsts
from drivetrain.drivetrainCommand import DrivetrainCommand
from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.ArmCommand import ArmCommand
from Elevatorandmech.RobotPoserCommon import PoseDirectorCommon
from wpimath.geometry import Pose2d
from positionSchemes._posintelligence import PlacementIntelligence
from humanInterface.operatorInterface import OperatorInterface, ReefLeftOrRight
from humanInterface.driverInterface import DriverInterface
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS
from wpimath.geometry import Pose2d

# if you can't find something here, it's probably in the _setup file.

class PlaceL4V6O(SetupScheme):
    def __init__(self, poseDirectorCommon: PoseDirectorCommon):
        super().__init__(arm=poseDirectorCommon.arm, base=poseDirectorCommon.driveTrain, elev=poseDirectorCommon.elevator)
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
        addLog("yvn_current_placeL4_state", lambda: self.currentState, "")
        addLog("yvn_placeL4_runs", lambda: self.totalRuns, "")
        self.placementIntel = PlacementIntelligence(self.base)
        # DEFINE THESE
        self.elevPlacePos = 60
        self.armPlacePos = 60

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:
                self.setElevatorCommand(self.elevPlacePos, 0)
                self.setArmCommand(self.armPlacePos, 0)
                if self.completedAwait("awaitForCmd1", 0.2):
                    self.nextState()
            case 1:
                elevGoalReached = math.isclose(self.elev.getPosition(), self.elevPlacePos, abs_tol=1)
                armGoalReached = math.isclose(self.arm.getPosition(), self.armPlacePos, abs_tol=2.5)
                self.localProg = sum([elevGoalReached, armGoalReached])/2
                if (elevGoalReached and armGoalReached) or self.oInt.skipNext:
                    self.nextState()
            case 3:
                pass
            case _:
                pass

        state_max = 3
        self.schemeProg = min((self.currentState+(self.localProg*0.9)) / state_max, 1)
