from enum import IntEnum
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController


from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS,\
MAX_ROTATE_SPEED_RAD_PER_SEC,MAX_TRANSLATE_ACCEL_MPS2,MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from utils.signalLogging import addLog
from utils.singleton import Singleton
# todo add intEnum class that indicates leftReef or rightReef

class ReefLeftOrRight(IntEnum):
    LEFT = 0
    RIGHT = 1
class ElevArmCmdState(IntEnum):
    VEL_CONTROL = 0
    PLUNGE = 2
    RECEIVE_CORAL = 3
    L1 = 4
    L2 = 5
    L3 = 6
    L4 = 7
    UNINITIALIZED = -1

class OperatorInterface(metaclass=Singleton):
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        # contoller
        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")

        #Noah
        #moved this to robotpy self.signalDirector = SignalDirector()
        #moved this to signalDirector self.robotPoser = RobotPoser()
        #self.currentState = self.signalDirector.getControllerState()
        self.elevArmCmdState = ElevArmCmdState.VEL_CONTROL


        # Drivetrain motion commands
        self.elevatorVelYCmd = 0
        self.armVelYCmd = 0

        # Driver motion rate limiters - enforce smoother driving
        #self.velXSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)

        #self.dPadState = ReefLeftOrRight.LEFT
        self.skipNext = False

        # Logging
        addLog("OI/elevatorVelCmd", lambda: self.elevatorVelYCmd, "frac")
        addLog("OI/elevator Pos In", lambda: self.getDesElevatorPosIn(), "in")
        addLog("OI/armVelCmd", lambda: self.armVelYCmd, "frac")
        addLog("OI/elevArmCmdState", lambda: self.elevArmCmdState, "int")




    def update(self):

        # value of contoller buttons

        if self.ctrl.isConnected():
            # Convert from  joystick sign/axis conventions to robot conventions
            leftYJoyRaw = self.ctrl.getLeftY() * -1
            rightYJoyRaw = self.ctrl.getRightY() * -1 # TODO xyzzy talk to Benjamin about a better name for this

            # deadband
            vElevJoyWithDeadband = applyDeadband(leftYJoyRaw, 0.05)
            vArmJoyWithDeadband = applyDeadband(rightYJoyRaw, 0.05) # TODO xyzzy talk to Benjamin about a better name for this

            slowMult = 1.0 if (self.ctrl.getRightBumper()) else 0.1
            
            self.elevatorVelYCmd = vElevJoyWithDeadband * slowMult
            self.armVelYCmd = vArmJoyWithDeadband * slowMult

            #print(f"elevatorVelYCmd={self.elevatorVelYCmd}")
            
            self.skipNext = self.ctrl.getBackButtonPressed()

            self.launchPlacement = self.ctrl.getLeftBumperPressed()

            if self.ctrl.getRightTriggerAxis() > .5:
                self.elevArmCmdState = ElevArmCmdState.PLUNGE
            elif self.ctrl.getLeftTriggerAxis() > .5:
                self.elevArmCmdState = ElevArmCmdState.RECEIVE_CORAL
            elif self.ctrl.getAButton():
                self.elevArmCmdState = ElevArmCmdState.L1
            elif self.ctrl.getXButton():
                self.elevArmCmdState = ElevArmCmdState.L2
            elif self.ctrl.getBButton():
                self.elevArmCmdState = ElevArmCmdState.L3
            elif self.ctrl.getYButton():
                self.elevArmCmdState = ElevArmCmdState.L4
            else:
                self.elevArmCmdState = ElevArmCmdState.VEL_CONTROL

        else:
            self.elevArmCmdState = ElevArmCmdState.UNINITIALIZED
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.elevatorVelYCmd = 0.0
            self.armVelYCmd = 0.0


# don't delete these they are unfortunately important (crying face emoji)

    def getDesElevatorPosIn(self)->float:
        elevatorRangeIn = 5.0
        return (elevatorRangeIn/2.0) * (1.0 + self.elevatorVelYCmd)

    def getDesArmAngleDeg(self)->float:
        return 90.0 * self.armVelYCmd

    def getElevArmCmdState(self)->ElevArmCmdState:
        return self.elevArmCmdState

    def getReefLeftOrRight(self)->ReefLeftOrRight:
        return self.dPadState


