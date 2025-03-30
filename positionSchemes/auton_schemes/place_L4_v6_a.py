import math
from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._intel._setup import SetupScheme, ArmConsts, ElevConsts
from positionSchemes._intel._posintelligence import PlacementIntelligence
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS

# multiply speed by 0.20 for the out speed

# if you can't find something here, it's probably in the _setup file.

class PlaceL4V1Auto(SetupScheme):
    def __init__(self, arm, base, elev, sideOfReef, reefTag):
        super().__init__(arm=arm, base=base, elev=elev)
        self.arm = arm
        self.base = base
        self.elev = elev
        self.pdSideOfReef = sideOfReef
        self.reefTag = reefTag
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

        self.odoPostEst = self.base.poseEst
        self.camPostEst = self.base.tcPoseEst

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:
                if self.completedAwait("waitForAutonLoad",2):
                    self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(7), self.reefTag)
                    print(f"place l4 time = {Timer.getFPGATimestamp():.3f}s x={self.bestTag.x:+10.1f}m y={self.bestTag.y:+10.1f}m t={self.bestTag.rotation().degrees():+10.1f}deg")
                    self.setDriveTrainBaseCommand(self.bestTag)
                    self.nextState()
            case 1:
                self.updateProgressTrajectory()
                self.setElevatorCommand(self.elevPlacePos, 0)
                self.setArmCommand(self.armPlacePos, 0)
                if self.completedTrajectory(self.base, 6, 5):
                    self.nextState()
            case 2:
                pass
            case 3:
                pass
            case 4:
                self.basePrimitiveCmd = None
                self.setArmCommand(47, -15)
                self.armPlacePos = 47
                armGoalReached = math.isclose(self.arm.getPosition(), self.armPlacePos, abs_tol=3)
                if armGoalReached:
                    self.nextState()
            case 5:
                self.setArmCommand(-15, 0)
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(20))
                self.setDriveTrainBaseCommand(self.bestTag)
                if self.completedTrajectory(self.base, 2, 4):
                    self.nextState()
            case 6:
                pass
            case _:
                pass

        state_max = 6
        self.schemeProg = min((self.currentState+(self.localProg*0.9)) / state_max, 1)
