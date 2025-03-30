import math
from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._intel._setup import SetupScheme, ArmConsts, ElevConsts
from positionSchemes._intel._posintelligence import PlacementIntelligence
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS


# if you can't find something here, it's probably in the _setup file.

class EmptyAuto(SetupScheme):
    def __init__(self, arm, base, elev):
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

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:
                print("running empty")
            case 1:
                pass
            case _:
                pass

        state_max = 1
        self.schemeProg = min((self.currentState+(self.localProg*0.9)) / state_max, 1)
