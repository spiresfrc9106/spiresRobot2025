from Elevatorandmech.armtest import ArmControl
from Elevatorandmech.elevatortest import ElevatorControl
from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._setup import SetupScheme, ArmConsts, ElevConsts
from drivetrain.drivetrainCommand import DrivetrainCommand
from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.ArmCommand import ArmCommand
from wpimath.geometry import Pose2d

#if you can't find something here, it's probably in the _setup file.

class Template(SetupScheme):
    def __init__(self, arm, base, elev):
        super().__init__(arm, base, elev)
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
        self.baseCmd = None
        self.armCmd = None
        self.elevCmd = None

        # structure:
        #   base: (Pose2d, velx, vely, velt)
        #   arm: (position_deg, deg/s)
        #   elev: (pasition_in, in/s)

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

        self.totalRuns = 0
        addLog("yvn_current_new_state", lambda: self.currentState, "")
        addLog("yvn_new_runs", lambda: self.totalRuns, "")

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0: #initializing
                pass
            case 1:
                pass
            case 2:
                pass
            case 3:
                pass
            case 4:
                pass
            case 5:
                pass
            case _:
                pass

        state_max = 5 #change for needs
        self.schemeProg = min(self.currentState/state_max, 1)
