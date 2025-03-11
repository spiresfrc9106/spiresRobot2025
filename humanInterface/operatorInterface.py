from enum import IntEnum
from debugpy.common.timestamp import current
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

class ElevArmCmdState(IntEnum):
    VEL_CONTROL = 0
    POS_CONTROL = 1
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
        self.elevatorPosYCmd = 0
        self.armPosYCmd = 0

        # Driver motion rate limiters - enforce smoother driving
        #self.velXSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)





        # Logging
        addLog("OI Elevator Pos Cmd", lambda: self.elevatorPosYCmd, "frac")
        addLog("OI Elevator Pos In", lambda: self.getDesElevatorPosIn(), "in")
        addLog("OI Arm Pos Cmd", lambda: self.armPosYCmd, "frac")
        addLog("armPosYCmd", lambda: self.armPosYCmd, "frac")


    def update(self):

        # value of contoller buttons

        if self.ctrl.isConnected():
            # Convert from  joystick sign/axis conventions to robot conventions
            vYJoyRaw = self.ctrl.getLeftY() * -1
            vYJoyRaw2 = self.ctrl.getRightY() * -1 # TODO xyzzy talk to Benjamin about a better name for this

            # deadband
            vYJoyWithDeadband = applyDeadband(vYJoyRaw, 0.15)
            vYJoy2WithDeadband = applyDeadband(vYJoyRaw2, 0.15) # TODO xyzzy talk to Benjamin about a better name for this

            self.elevatorPosYCmd = vYJoyWithDeadband
            self.armPosYCmd = vYJoy2WithDeadband

            if self.ctrl.getRightBumperPressed():
                self.elevArmCmdState = ElevArmCmdState.VEL_CONTROL  # xyzzy redundant because this is also the default
            elif self.ctrl.getLeftBumperPressed():
                self.elevArmCmdState = ElevArmCmdState.POS_CONTROL
            elif self.ctrl.getRightTriggerAxis() > .5:
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
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.elevatorPosYCmd = 0.0
            self.armPosYCmd = 0.0



    def getDesElevatorPosIn(self)->float:
        elevatorRangeIn = 5.0
        return (elevatorRangeIn/2.0) * (1.0 + self.elevatorPosYCmd)

    def getDesArmAngleDeg(self)->float:
        return 90.0 * self.armPosYCmd

    def getElevArmCmdState(self)->ElevArmCmdState:
        return self.elevArmCmdState


