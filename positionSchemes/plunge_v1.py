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

from utils.units import deg2Rad


# if you can't find something here, it's probably in the _setup file.

class PlungeV1(SetupScheme):
    def __init__(self, arm, base, elev, oInt):
        super().__init__(arm, base, elev)
        self.arm = arm
        self.base = base
        self.elev = elev
        self.armConst = ArmConsts()
        self.elevConst = ElevConsts()
        self.currentState = 0
        self.oInt = oInt

        # structure:
        #   base: (Pose2d, velx, vely, velt)
        #   arm: (position_deg, deg/s)
        #   elev: (position_in, in/s)

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

        # DOWN HEIGHT
        self.elevSafestPlungeHeight_in = (43.47 + 41) / 2  # or 41.5? # 40.91 - 1 #added -1 #32.1875 - 2 - 1
        
        # UP HEIGHT
        self.elevBestMediumHeight_in = self.elevConst.posMedium
        
        # RESET HEIGHT
        self.axisToCenterOfCoral_in = 22.5
        # pivot is 8 inches from the edge/center of the robot
        self.elevSafestResetHeight_in = self.elevSafestPlungeHeight_in + self.axisToCenterOfCoral_in

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        self.setDriveTrainBaseCommand(None)  # because this never changes, we're not going to bother with it.
        match self.currentState:
            case 0:  # initializing
                if self.elev.getPosition() < self.elevSafestPlungeHeight_in + 9:
                    self.elevCmd = (self.elevSafestPlungeHeight_in + 10, 0)
                else:
                    self.nextState()
            case 1:
                self.armCmd = (-90, 0)  # lowest and 0 speed when reached
                self.elevCmd = None  # originally we were going to slowly bring this down but no.
                if math.isclose(self.arm.getPosition(), -90, abs_tol=2.5):
                    self.nextState()
            case 2:  #
                self.armCmd = (-90, 0)
                self.elevCmd = (self.elevSafestPlungeHeight_in, 0)
                if math.isclose(self.elev.getPosition(), self.elevSafestPlungeHeight_in, abs_tol=0.5):
                    self.nextState()
            case 3:
                if self.completedAwait("bottomWait", 0.5):
                    self.nextState()
            case 4:
                self.elevCmd = (self.elevBestMediumHeight_in, 0)
                if self.completedAwait("waterfallElevUp", 0.1):
                    self.nextState()
            case 5:
                if self.elev.getPosition() > self.elevBestMediumHeight_in-1:
                    self.armCmd = (90, 0)
                if math.isclose(self.arm.getPosition(), 90, abs_tol=2):
                    self.nextState()
            case 6:
                pass
            case _:
                pass

        state_max = 6
        self.schemeProg = min((self.currentState+(self.localProg*0.9)) / state_max, 1)
