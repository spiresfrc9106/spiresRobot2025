import math

from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._setup import SetupScheme, ArmConsts, ElevConsts
from positionSchemes.RobotPoserCommon import PoseDirectorCommon
from positionSchemes._posintelligence import PlacementIntelligence
from humanInterface.operatorInterface import ReefLeftOrRight
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS


# if you can't find something here, it's probably in the _setup file.

class PlaceL2V1(SetupScheme):
    def __init__(self, poseDirectorCommon: PoseDirectorCommon):
        super().__init__(arm=poseDirectorCommon.arm, base=poseDirectorCommon.driveTrain, elev=poseDirectorCommon.elevator)
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
        addLog("yvn_current_placeL2_state", lambda: self.currentState, "")
        addLog("yvn_placeL2_runs", lambda: self.totalRuns, "")
        self.placementIntel = PlacementIntelligence(self.base)
        # DEFINE THESE
        self.elevPlacePos = 18
        self.elevDistanceDownNeeded = 6  # just measure the distance from one little thing to the one below.
        self.armPlacePos = 55

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:  # initializing
                # self.armCmd/elevCmd could be called here to prep for the fun thing.
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(11))
                self.setDriveTrainBaseCommand(self.bestTag)
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
                self.setDriveTrainBaseCommand(self.bestTag)
                if self.completedAwait("waitforcmdsend1", 0.2):
                    self.nextState()
            case 3:
                elevGoalReached = math.isclose(self.elev.getPosition(), self.elevPlacePos, abs_tol=0.5)
                armGoalReached = math.isclose(self.arm.getPosition(), self.armPlacePos, abs_tol=5)  # is abs_tol# good?
                baseGoalReached = self.completedTrajectory(self.base)
                if elevGoalReached and armGoalReached and baseGoalReached:
                    self.nextState()
                else:
                    self.localProg = sum([elevGoalReached, armGoalReached, baseGoalReached])/3
            case 4: #human input
                elevCmdOIntNorm = self.oInt.elevatorVelYCmd * 10
                armCmdOIntNorm = self.oInt.armVelYCmd * 30
                xCmdDIntNorm = self.dInt.getVelXCmd() * MAX_FWD_REV_SPEED_MPS
                yCmdDIntNorm = self.dInt.getVelYCmd() * MAX_FWD_REV_SPEED_MPS
                elevCmdOIntNew = elevCmdOIntNorm*0.02
                armCmdOIntNew = armCmdOIntNorm*0.02
                xCmdDIntNew = xCmdDIntNorm*0.02
                yCmdDIntNew = yCmdDIntNorm*0.02
                self.armCmd = (None, armCmdOIntNew)
                self.elevCmd = (None, elevCmdOIntNew)
                self.basePrimitiveCmd = (xCmdDIntNew, yCmdDIntNew, 0)
                if self.oInt.launchPlacement:
                    self.nextState()
            case 5: #launch it bruh
                self.basePrimitiveCmd = None
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(3.5))
                self.setDriveTrainBaseCommand(self.bestTag)
                if self.completedAwait("delayfornodecapture",0.4):
                    self.nextState()
            case 6:
                self.elevCmd = (self.elevPlacePos-self.elevDistanceDownNeeded, 0)
                if self.completedTrajectory(self.base):
                    self.nextState()
            case 7:
                elevGoal = self.elevPlacePos-self.elevDistanceDownNeeded
                elevGoalReached = math.isclose(self.elev.getPosition(), elevGoal, abs_tol=1) # is abs_tol# good?
                if elevGoalReached:
                    self.nextState()
            case 8:
                pass
            case _:
                pass

        state_max = 8
        # when calculating the scheme prog, we can also add in local progress to show something as we go thru state.
        self.schemeProg = min((self.currentState+self.localProg) / (state_max), 1)
