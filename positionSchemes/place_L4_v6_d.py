from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._setup import SetupScheme, ArmConsts, ElevConsts
from positionSchemes.RobotPoserCommon import PoseDirectorCommon
from positionSchemes._posintelligence import PlacementIntelligence
from humanInterface.operatorInterface import OperatorInterface, ReefLeftOrRight
from humanInterface.driverInterface import DriverInterface


# if you can't find something here, it's probably in the _setup file.

class PlaceL4V6D(SetupScheme):
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
        addLog("yvn_current_placeL4_state", lambda: self.currentState, "")
        addLog("yvn_placeL4_runs", lambda: self.totalRuns, "")
        self.placementIntel = PlacementIntelligence(self.base)

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, self.inchesToMeters(7))
                print(f"place l4 time = {Timer.getFPGATimestamp():.3f}s x={self.bestTag.x:+10.1f}m y={self.bestTag.y:+10.1f}m t={self.bestTag.rotation().degrees():+10.1f}deg")
                self.setDriveTrainBaseCommand(self.bestTag)
                self.nextState()
            case 1:
                self.updateProgressTrajectory()
                if self.completedTrajectory(self.base, 1, 1) or self.dInt.skipNext:
                    self.nextState()
            case 2:
                pass
            case _:
                pass

        state_max = 2
        self.schemeProg = min((self.currentState+(self.localProg*0.9)) / state_max, 1)
