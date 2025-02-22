from debugpy.common.timestamp import current
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController

import Elevatorandmech.RobotPoser
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS,\
MAX_ROTATE_SPEED_RAD_PER_SEC,MAX_TRANSLATE_ACCEL_MPS2,MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from utils.signalLogging import addLog
from Elevatorandmech.RobotPoser import SignalDirector, RobotPoser
from Elevatorandmech.ElevatorControl import ElevatorControl


class OperatorInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        # contoller
        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Operator XBox controller ({ctrlIdx}) unplugged")

        #Noah
        self.signalDirector = SignalDirector()
        self.robotPoser = RobotPoser()
        self.currentState = self.signalDirector.getControllerState()

        self.elevatorControl = ElevatorControl()

        # Drivetrain motion commands
        self.elevatorPosYCmd = 0

        # Driver motion rate limiters - enforce smoother driving
        #self.velXSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)





        # Logging
        addLog("OI Elevator Pos Cmd", lambda: self.elevatorPosYCmd, "frac")


    def update(self):
        # value of contoller buttons
        self.currentState = self.signalDirector.getControllerState()

        if self.ctrl.isConnected():
            # Convert from  joystick sign/axis conventions to robot conventions
            vYJoyRaw = self.ctrl.getLeftY() * -1

            # deadband
            vYJoyWithDeadband = applyDeadband(vYJoyRaw, 0.15)

            self.elevatorPosYCmd = vYJoyWithDeadband

        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.elevatorPosYCmd = 0.0

        #--------------------------------------------------------------------------------------------------
        #Noahw

        #Defaults to Velocity movement state or position movement state if no buttons are pressed
        self.signalDirector.setControllerState(self.signalDirector.getDefaultJoystickMovement())


        #Position Mode
        if self.ctrl.getLeftBumperPressed():
            self.signalDirector.setControllerState("PositionControl")

        #Velocity Mode
        if self.ctrl.getRightBumperPressed():
            self.signalDirector.setControllerState("VelocityControl")

        #Plunge
        if self.ctrl.getYButton():
            self.signalDirector.setControllerState("Plunge")




    def getDesElevatorPosIn(self)->float:
        elevatorRangeIn = 6.0
        elevatorCurrentHeight = self.elevatorControl.getHeightIn()

        #Disables Joysticks If not in position or velocity movement states
        if not self.currentState == 0 and not self.currentState == 1:
            self.elevatorPosYCmd = self.signalDirector.disableJoySticks()


        if self.currentState == 0:

            return (elevatorRangeIn/2.0) * (1.0 + self.elevatorPosYCmd)

        elif self.currentState == 1:

            if self.elevatorPosYCmd > 0:
                return elevatorCurrentHeight + 0.5
            elif self.elevatorPosYCmd < 0:
                return elevatorCurrentHeight - 0.5
            else:
                return elevatorCurrentHeight
        elif self.currentState == 2:
            pass
        elif self.currentState == 3:
            return RobotPoser.getPlungeElevatorHeight()


