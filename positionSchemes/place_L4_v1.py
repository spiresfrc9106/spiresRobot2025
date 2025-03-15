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

# if you can't find something here, it's probably in the _setup file.

class PlaceL4V1(SetupScheme):
    def __init__(self, arm, base, elev, oInt):
        super().__init__(arm, base, elev)
        self.arm = arm
        self.base = base
        self.elev = elev
        self.oInt = oInt
        self.dInt = DriverInterface()
        self.pdReefSideState = self.oInt.getReefLeftOrRight()
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
        self.baseCmd = None
        self.armCmd = None
        self.elevCmd = None

        # structure:
        #   base: (Pose2d, velx, vely, velt)
        #   arm: (position_deg, deg/s)
        #   elev: (pasition_in, in/s)

        # PROCESS:
        # in
        #  1) find the closest april tag designated as pickup (ignore grid system idea)
        #  -- ignore the camera system to pick the best place to go to... we'll do this later.
        #     -> based on the arrow on which side to go to (determined by the class but currently user input)

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
            case 0:  # initializing
                # self.armCmd/elevCmd could be called here to prep for the fun thing.
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(11))
                self.baseCmd = (self.bestTag, 0, 0, 0)
                # CAN WE DO BETTER?  YES OF COURSE WE CAN.
                self.armCmd = (90, 0)  # straight up so no bumping.
                if self.completedAwait("awaitbasecmdsend", 0.2):
                    self.nextState()
            case 1:
                if self.completedTrajectory(self.base):
                    self.nextState()
                pass
            case 2:
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(7))
                self.elevCmd = (self.elevPlacePos, 0)
                self.armCmd = (self.armPlacePos, 0)
                self.baseCmd = (self.bestTag, 0, 0, 0)
                if self.completedAwait("waitforcmdsend1", 0.2):
                    self.nextState()
            case 3:
                elevGoalReached = math.isclose(self.elev.getPosition(), self.elevPlacePos, abs_tol=0.5)
                armGoalReached = math.isclose(self.arm.getPosition(), self.armPlacePos, abs_tol=5) #idk if this'll work
                baseGoalReached = self.completedTrajectory(self.base)
                if elevGoalReached and armGoalReached and baseGoalReached:
                    self.nextState()
            case 4: #human input
                elevCmdOIntNorm = self.oInt.elevatorPosYCmd * 10
                armCmdOIntNorm = self.oInt.armPosYCmd * 30
                xCmdDIntNorm = self.dInt.getVelXCmd() * MAX_FWD_REV_SPEED_MPS
                yCmdDIntNorm = self.dInt.getVelYCmd() * MAX_FWD_REV_SPEED_MPS
                elevCmdOIntNew = elevCmdOIntNorm*0.25
                armCmdOIntNew = armCmdOIntNorm*0.25
                xCmdDIntNew = xCmdDIntNorm*0.25
                yCmdDIntNew = yCmdDIntNorm*0.25
                self.armCmd = (None, armCmdOIntNew)
                self.elevCmd = (None, elevCmdOIntNew)
                self.basePrimitiveCmd = (xCmdDIntNew, yCmdDIntNew, 0)
                if self.oInt.launchPlacement:
                    self.nextState()
            case 5: #launch it bruh
                self.basePrimitiveCmd = None
                self.armCmd = (50, -15)
                armGoalReached = math.isclose(self.arm.getPosition(), self.armPlacePos, abs_tol=0.75)
                if armGoalReached:
                    self.nextState()
            case 6:
                self.armCmd = (-15, 0)
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(10))
                self.baseCmd = (self.bestTag, 0, 0, 0)
                if self.completedTrajectory(self.base):
                    self.nextState()
            case 7:
                pass
            case _:
                pass

        state_max = 7
        # when calculating the scheme prog, we can also add in local progress to show something as we go thru state.
        self.schemeProg = min((self.currentState) / (state_max), 1)
