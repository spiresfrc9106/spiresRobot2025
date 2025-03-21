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
from positionSchemes._posintelligence import PickupIntelligence

#if you can't find something here, it's probably in the _setup file.

class PickupV1(SetupScheme):
    def __init__(self, arm, base, elev, oInt):
        super().__init__(arm, base, elev)
        self.arm = arm
        self.base = base
        self.elev = elev
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
        #   elev: (pasition_in, in/s)

        #PROCESS:
        # in
        #  1) find the closest april tag designated as pickup (ignore grid system idea)
        #  -- ignore the camera system to pick the best place to go to... we'll do this later.
        #     -> based on the arrow on which side to go to (determined by the class but currently user input)


        self.totalRuns = 0
        self.bestTag = 0
        addLog("yvn_current_pickup_state", lambda: self.currentState, "")
        addLog("yvn_pickup_runs", lambda: self.totalRuns, "")

        self.elevPickupPose = 47.6875  #inches

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0: #initializing
                #self.armCmd/elevCmd could be called here to prep for the fun thing.
                self.bestTag = PickupIntelligence(self.base).decidePickupPose(self.inchesToMeters(10))
                self.setDriveTrainBaseCommand(self.bestTag)
                self.armCmd = None
                self.elevCmd = None
                # CAN WE DO BETTER?  YES OF COURSE WE CAN.
                # one thing that comes to mind is perhaps breaking down
                # 1) long commutes or 2) big rotation distances
                # into two or more seperate trajectory commands.
                # however, since this won't be as likely on the field,
                # we're leaving it here
                if self.completedAwait("awaitbasecmdsend", 0.2):
                    self.nextState()
            case 1:
                self.elevCmd = (self.elevPickupPose, 0)
                if self.completedTrajectory(self.base):
                    self.nextState()
            case 2:
                elevGoalReached = math.isclose(self.elev.getPosition(), self.elevPickupPose, abs_tol=0.5)
                if elevGoalReached:
                    self.nextState()
            case 3:
                self.bestTag = PickupIntelligence(self.base).decidePickupPose(self.inchesToMeters(1))
                self.setDriveTrainBaseCommand(self.bestTag)
                pass
            case _:
                pass

        state_max = 3
        # when calculating the scheme prog, we can also add in local progress to show something as we go thru state.
        self.schemeProg = min((self.currentState) / (state_max), 1)
