from Elevatorandmech.armtest import ArmControl
from Elevatorandmech.elevatortest import ElevatorControl
from wpilib import Timer
from utils.signalLogging import addLog

class PlungeV1():
    def __init__(self):
        self.arm = ArmControl()
        self.elev = ElevatorControl()

        #PROCESS:
        #  base: no action
        #  starting: arm dnm, elevator high enough to come down
        #  1) arm down (fast)
        #  await down
        #  2) elevator down (fast)
        #  await down
        #  3) wait 0.5s
        #  4) elevator up (medium v)
        #  5) arm up (medium v), to a medium point (medium raised)

        self.baseCmd = None
        self.armCmd = None
        self.elevCmd = None

        self.startTime = Timer.getFPGATimestamp()
        self.changeInTime = 0

        self.waitTimes = {}
        self.currentState = 0
        self.schemeProg = 0

        self.armConst = ArmConsts()
        self.elevConst = ElevConsts()

        self.totalRuns = 0

        addLog("yvn_current_plunge_state", lambda: self.currentState, "")
        addLog("yvn_runs", lambda: self.totalRuns, "") # test purposes, not needed at all.

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        self.baseCmd = None # because this never changes, we're not going to bother with it.
        match self.currentState:
            case 0: #initializing
                self.armCmd = None
                self.elevCmd = (self.elevConst.posMedium, -1 * self.elevConst.velHigh)
                if self.elev.atAboutMiddle: #this is in case the elev was at the bottom
                    self.nextState()
            case 1: #moving elevator in negative direction, not intending to hit bottom
                self.armCmd = (self.armConst.posLow, 0)
                self.elevCmd = (self.elevConst.posLowMid, -1 * self.elevConst.velMedium)
                if self.arm.atAboutDown:
                    self.nextState()
            case 2: #
                self.armCmd = (self.armConst.posLow, 0)
                self.elevCmd = (self.elevConst.posLow, 0)
                if self.elev.reachedBottom:
                    self.nextState()
            case 3:
                if self.completedAwait("bottomWait",0.5):
                    self.nextState()
            case 4:
                self.armCmd = (self.armConst.posHigh, 0)
                if self.completedAwait("waterfallArmUp",0.1):
                    self.nextState()
            case 5:
                self.armCmd = (self.armConst.posHigh, 0)
                self.elevCmd = (self.elevConst.posMedium, 0)
            case _:
                pass

        state_max = 5
        self.schemeProg = min(self.currentState/state_max, 1)

    def nextState(self):
        self.currentState = self.currentState + 1

    def completedAwait(self, waitName, duration):
        now = Timer.getFPGATimestamp()
        if self.waitTimes.get(waitName, -1) == -1:
            self.waitTimes[waitName] = now
        if now - self.waitTimes[waitName] >= duration:
            return True
        else:
            return False


class ArmConsts:
    def __init__(self):
        self.posLow = 1
        self.posMedium = 2
        self.posHigh = 3
        self.velLow = 1
        self.velMedium = 2
        self.velHigh = 3


class ElevConsts:
    def __init__(self):
        self.posLow = 1
        self.posLowMid = 1.5 # this is the location that the robot should be right before hitting coral
        self.posMedium = 2 # middle of elevator, default location.
        self.posHigh = 3
        self.velLow = 1
        self.velMedium = 2
        self.velHigh = 3

