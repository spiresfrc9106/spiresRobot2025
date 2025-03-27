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

# if you can't find something here, it's probably in the _setup file.

class PlaceL4V6(SetupScheme):
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
        addLog("yvn_current_putL4_d_state", lambda: self.currentState, "")
        self.placementIntel = PlacementIntelligence(self.base)

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:
                self.basePrimitiveCmd = None
                if self.oInt.skipNext:
                    self.nextState()
            case 1:
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(20))
                self.setDriveTrainBaseCommand(self.bestTag)
                self.updateProgressTrajectory()
                if self.completedTrajectory(self.base, 2, 4) or self.oInt.skipNext:
                    self.nextState()
            case 2:
                pass
            case _:
                pass

        state_max = 2
        self.schemeProg = min((self.currentState+(self.localProg*0.9)) / state_max, 1)
