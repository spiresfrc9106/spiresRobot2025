from Elevatorandmech.armtest import ArmControl
from Elevatorandmech.elevatortest import ElevatorControl
from wpilib import Timer
from utils.signalLogging import addLog
from positionSchemes._setup import SetupScheme, ArmConsts, ElevConsts
from drivetrain.drivetrainCommand import DrivetrainCommand
from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.ArmCommand import ArmCommand
from wpimath.geometry import Pose2d
from positionSchemes._posintelligence import PlacementIntelligence


# if you can't find something here, it's probably in the _setup file.

class PlaceL4V1(SetupScheme):
    def __init__(self, arm, base, elev):
        super().__init__(arm, base, elev)
        self.arm = arm
        self.base = base
        self.elev = elev
        #self.pd !!!!
        self.pdSideOfReef = -1
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

        # PROCESS:
        # in
        #  1) find the closest april tag designated as pickup (ignore grid system idea)
        #  -- ignore the camera system to pick the best place to go to... we'll do this later.
        #     -> based on the arrow on which side to go to (determined by the class but currently user input)

        self.totalRuns = 0
        self.bestTag = 0
        addLog("yvn_current_placeL4_state", lambda: self.currentState, "")
        addLog("yvn_placeL4_runs", lambda: self.totalRuns, "")
        self.placementIntel = PlacementIntelligence(self.base)
        # DEFINE THESE
        self.elevPlacePos = 0
        self.armPlacePos = 0

    def update(self):
        currentTime = Timer.getFPGATimestamp()
        time = currentTime - self.startTime
        match self.currentState:
            case 0:  # initializing
                # self.armCmd/elevCmd could be called here to prep for the fun thing.
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef,0.001)
                self.baseCmd = (self.bestTag, 0, 0, 0)
                # CAN WE DO BETTER?  YES OF COURSE WE CAN.
                self.armCmd = (90, 0)  # straight up so no bumping.
                if self.completedAwait("awaitbasecmdsend", 0.2):
                    self.nextState()
            case 1:
                if self.completedTrajectory(self.base):
                    self.nextState()
                pass
            case 2:
                # bring elevator to the height needed to place.
                # bring arm to the angle needed to place
                # check for completed await of 0.2 seconds
                #    nextstate
                pass
            case 3:
                # if elev.pos==desired and if arm.pos==desired:
                #  nextState
                pass
            case 4:
                # should we actually launch?

                # do we need a new location?:
                self.bestTag = self.placementIntel.decidePlacementPose(self.pdSideOfReef, 0.001)
                self.baseCmd = (self.bestTag, 0, 0, 0)
            case _:
                pass

        state_max = 4
        self.schemeProg = min(self.currentState / state_max, 1)
