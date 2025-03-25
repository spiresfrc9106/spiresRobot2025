import math
from Elevatorandmech.armtest import ArmControl
from Elevatorandmech.elevatortest import ElevatorControl
from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._setup import SetupScheme, ArmConsts, ElevConsts
from drivetrain.drivetrainCommand import DrivetrainCommand
from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.ArmCommand import ArmCommand
from wpimath.geometry import Pose2d
from positionSchemes._posintelligence import PlacementIntelligence
from humanInterface.operatorInterface import OperatorInterface, ReefLeftOrRight
from humanInterface.driverInterface import DriverInterface
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS
from wpimath.geometry import Pose2d
from drivetrain.DrivetrainDependentConstants import drivetrainDepConstants

# if you can't find something here, it's probably in the _setup file.

class PlaceL4V3(SetupScheme):
    def __init__(self, arm, base, elev, oInt):
        super().__init__(arm=arm, base=base, elev=elev)
        self.arm = arm
        self.base = base
        self.elev = elev
        self.oInt: OperatorInterface = oInt
        self.dInt = DriverInterface()
        self.pdReefSideState = self.oInt.dPadState
        self.pdSideOfReef = -1
        if self.pdReefSideState == ReefLeftOrRight.RIGHT:
            self.pdSideOfReef = 1
        self.armConst = ArmConsts()
        self.elevConst = ElevConsts()
        self.currentState = 0
        self.oInt = oInt

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
        ############TODO in postEst and tcposeEst, add in latencies from LL.

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:  # initializing
                # self.armCmd/elevCmd could be called here to prep for the fun thing.
                self.bestTag = self.placementIntel.decidePlacementPose(0, self.inchesToMeters(21))
                print(f"place l4 time = {Timer.getFPGATimestamp():.3f}s x={self.bestTag.X():+10.1f}m y={self.bestTag.Y():+10.1f}m t={self.bestTag.rotation().degrees():+10.1f}deg")
                self.setDriveTrainBaseCommand(self.bestTag)
                # self.armCmd = (90, 0)  # straight up so no bumping.
                if self.completedAwait("awaitBaseCmdRecognition", 0.2):
                    self.nextState()
            case 1:
                self.updateProgressTrajectory()
                if self.completedTrajectory(self.base, 4, 3) or self.oInt.skipNext:
                    self.nextState()
            case 2:
                self.armCmd = (-15, 0)
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(0))
                self.setDriveTrainBaseCommand(self.bestTag)
                self.updateProgressTrajectory()
                if self.completedTrajectory(self.base) or self.oInt.skipNext:
                    self.nextState()
            case 3:
                self.setDriveTrainBaseCommand(None)
            case _:
                pass

        state_max = 3
        # when calculating the scheme prog, we can also add in local progress to show something as we go thru state.
        self.schemeProg = min((self.currentState+(self.localProg*0.9)) / state_max, 1)
