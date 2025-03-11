from Elevatorandmech.armtest import ArmControl
from Elevatorandmech.elevatortest import ElevatorControl
from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._setup import SetupScheme, ArmConsts, ElevConsts
from drivetrain.drivetrainCommand import DrivetrainCommand
from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.ArmCommand import ArmCommand
from wpimath.geometry import Pose2d


# if you can't find something here, it's probably in the _setup file.

class PlungeV1(SetupScheme):
    def __init__(self, arm, base, elev):
        super().__init__(arm, base, elev)
        self.arm = arm
        self.base = base
        self.elev = elev
        self.armConst = ArmConsts()
        self.elevConst = ElevConsts()
        self.currentState = 0

        # structure:
        #   base: (Pose2d, velx, vely, velt)
        #   arm: (position_deg, deg/s)
        #   elev: (pasition_in, in/s)

        # PROCESS:
        #  base: no action
        #  starting: arm dnm, elevator high enough to come down
        #  1) arm down (fast)
        #  await down
        #  2) elevator down (fast)
        #  await down
        #  3) wait 0.5s
        #  4) elevator up (medium v)
        #  5) arm up (medium v), to a medium point (medium raised)

        self.totalRuns = 0
        addLog("yvn_current_plunge_state", lambda: self.currentState, "")
        addLog("yvn_plunge_runs", lambda: self.totalRuns, "")  # test purposes, not needed at all.

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        self.baseCmd = None  # because this never changes, we're not going to bother with it.
        #
        #
        #
        #  THE WHOLE THING HERE IS INCORRECT SEQUENCING. PLEASE REVISE FOR FUNCTIONALITY AND SAFETY.
        #
        #
        #
        match self.currentState:
            case 0:  # initializing
                self.armCmd = None
                self.elevCmd = (self.elevConst.posMedium, -1 * self.elevConst.velHigh)
                if self.elev.atAboutMiddle:  # this is in case the elev was at the bottom
                    self.nextState()
            case 1:  # moving elevator in negative direction, not intending to hit bottom
                self.armCmd = (self.armConst.posLow, 0)
                self.elevCmd = (self.elevConst.posLowMid, -1 * self.elevConst.velMedium)
                if self.arm.atAboutDown:
                    self.nextState()
            case 2:  #
                self.armCmd = (self.armConst.posLow, 0)
                self.elevCmd = (self.elevConst.posLow, 0)
                if self.elev.reachedBottom:
                    self.nextState()
            case 3:
                if self.completedAwait("bottomWait", 0.5):
                    self.nextState()
            case 4:
                self.armCmd = (self.armConst.posHigh, 0)
                if self.completedAwait("waterfallArmUp", 0.1):
                    self.nextState()
            case 5:
                self.armCmd = (self.armConst.posHigh, 0)
                self.elevCmd = (self.elevConst.posMedium, 0)
            case _:
                pass

        state_max = 5
        self.schemeProg = min(self.currentState / state_max, 1)
