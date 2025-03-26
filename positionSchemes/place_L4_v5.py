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

class PlaceL4V5(SetupScheme):
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

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:
                self.bestTag = self.placementIntel.decidePlacementPose(0, self.inchesToMeters(20))
                print(f"place l4 time = {Timer.getFPGATimestamp():.3f}s x={self.bestTag.x:+10.1f}m y={self.bestTag.y:+10.1f}m t={self.bestTag.rotation().degrees():+10.1f}deg")
                self.setDriveTrainBaseCommand(self.bestTag)
                self.setArmCommand(90, 0)
                self.updateProgressTrajectory()
                if self.completedTrajectory(self.base, 6, 5) or self.oInt.skipNext:
                    self.nextState()
            case 1:
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(7))
                self.setElevatorCommand(self.elevPlacePos, 0)
                self.setArmCommand(self.armPlacePos, 0)
                self.setDriveTrainBaseCommand(self.bestTag)
                if self.completedAwait("awaitForCmd2", 0.2):
                    self.nextState()
            case 2:
                elevGoalReached = math.isclose(self.elev.getPosition(), self.elevPlacePos, abs_tol=1) #og 0.5
                armGoalReached = math.isclose(self.arm.getPosition(), self.armPlacePos, abs_tol=2.5)  # is abs_tol good?
                baseGoalReached = self.completedTrajectory(self.base, 0.75, 3)
                if (elevGoalReached and armGoalReached and baseGoalReached) or self.oInt.skipNext:
                    self.nextState()
                else:
                    self.localProg = sum([elevGoalReached, armGoalReached, baseGoalReached])/3
            case 3:
                elevCmdOIntNorm = self.oInt.elevatorVelYCmd * 10
                armCmdOIntNorm = self.oInt.armVelYCmd * 30
                xCmdDIntNorm = self.dInt.getVelXCmd() * MAX_FWD_REV_SPEED_MPS
                yCmdDIntNorm = self.dInt.getVelYCmd() * MAX_FWD_REV_SPEED_MPS
                elevCmdOIntNew = elevCmdOIntNorm*0.5
                armCmdOIntNew = armCmdOIntNorm*0.5
                xCmdDIntNew = xCmdDIntNorm*0.25
                yCmdDIntNew = yCmdDIntNorm*0.25
                self.setArmCommand(None, armCmdOIntNew)
                self.setElevatorCommand(None, elevCmdOIntNew)
                self.basePrimitiveCmd = (xCmdDIntNew, yCmdDIntNew, 0)
                if self.oInt.launchPlacement:
                    self.nextState()
            case 4:
                self.basePrimitiveCmd = None
                self.setArmCommand(47, -15)
                self.armPlacePos = 47
                armGoalReached = math.isclose(self.arm.getPosition(), self.armPlacePos, abs_tol=3)
                if armGoalReached or self.oInt.skipNext:
                    self.nextState()
            case 5:
                self.setArmCommand(-15, 0)
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(20))
                self.setDriveTrainBaseCommand(self.bestTag)
                if self.completedTrajectory(self.base, 2, 4) or self.oInt.skipNext:
                    self.nextState()
            case 6:
                pass
            case _:
                pass

        state_max = 6
        self.schemeProg = min((self.currentState+(self.localProg*0.9)) / state_max, 1)
