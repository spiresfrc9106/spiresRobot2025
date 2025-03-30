import math
from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._intel._setup import SetupScheme, ArmConsts, ElevConsts
from positionSchemes._intel._posintelligence import PlacementIntelligence, YPose
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS
from wpimath.geometry import Pose2d

# multiply speed by 0.20 for the out speed

# if you can't find something here, it's probably in the _setup file.

class DriveFwdAuto(SetupScheme):
    def __init__(self, arm, base, elev, side, distance):
        super().__init__(arm=arm, base=base, elev=elev)
        self.arm = arm
        self.base = base
        self.elev = elev
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
        self.placementIntel = PlacementIntelligence(self.base)
        # DEFINE THESE
        self.elevPlacePos = 60
        self.armPlacePos = 60

        self.odoPostEst = self.base.poseEst
        self.camPostEst = self.base.tcPoseEst

        self.ogPos = 0
        self.driveOut_m = side * distance

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:
                self.ogPos = YPose(self.odoPostEst.getCurEstPose())
                self.locationGoal = Pose2d(self.ogPos.x+self.driveOut_m, self.ogPos.y, self.ogPos.t)
                self.setDriveTrainBaseCommand(self.locationGoal)
                self.updateProgressTrajectory()
                self.nextState()
            case 1:
                pass
            case _:
                pass

        state_max = 1
        self.schemeProg = min((self.currentState+(self.localProg*0.9)) / state_max, 1)
