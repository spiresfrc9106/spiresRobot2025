from pint.testsuite.test_application_registry import custom_registry

from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS,\
MAX_ROTATE_SPEED_RAD_PER_SEC,MAX_TRANSLATE_ACCEL_MPS2,MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from utils.signalLogging import addLog
from utils.units import deg2Rad
from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController
from drivetrain.drivetrainControl import DrivetrainControl


class DriverInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        self.poseEst = DrivetrainControl().poseEst
        #rawGyroReading = self.poseEst.getCurEstPose().rotation().degrees() to convert to what we want t osee
        # contoller
        ctrlIdx = 0
        self.ctrl = XboxController(ctrlIdx)
        self.connectedFault = Fault(f"Driver XBox controller ({ctrlIdx}) unplugged")

        # Drivetrain motion commands
        self.velXCmd = 0
        self.velYCmd = 0
        self.velTCmd = 0

        # Driver motion rate limiters - enforce smoother driving
        self.velXSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)
        self.velYSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_TRANSLATE_ACCEL_MPS2)
        self.velTSlewRateLimiter = SlewRateLimiter(rateLimit=MAX_ROTATE_ACCEL_RAD_PER_SEC_2)

        # Navigation commands
        self.autoDriveToSpeaker = False
        self.autoDriveToPickup = False
        self.createDebugObstacle = False

        # Utility - reset to zero-angle at the current pose
        self.gyroResetCmd = False

        # Logging
        #addLog("DI FwdRev Cmd", lambda: self.velXCmd, "mps")
        #addLog("DI Strafe Cmd", lambda: self.velYCmd, "mps")
        #addLog("DI Rot Cmd", lambda: self.velTCmd, "radps")
        #addLog("DI gyroResetCmd", lambda: self.gyroResetCmd, "bool")
        #addLog("DI autoDriveToSpeaker", lambda: self.autoDriveToSpeaker, "bool")
        #addLog("DI autoDriveToPickup", lambda: self.autoDriveToPickup, "bool")

    def degreesTillDesiredGyro(self, desiredGyro):
        normalizedGyro = self.normalizeGyroAngle()
        normalizedAndDesiredAngleDifference = normalizedGyro - desiredGyro

        if normalizedAndDesiredAngleDifference > 180:
            return (360-normalizedGyro)+desiredGyro
        elif 0<normalizedAndDesiredAngleDifference<=180:
            return normalizedGyro - desiredGyro
        elif -180<normalizedAndDesiredAngleDifference<0:
            return abs(normalizedAndDesiredAngleDifference)
        else:
            return 360+(normalizedAndDesiredAngleDifference)

    def normalizeGyroAngle(self):
        rawGyroReading = self.poseEst.getCurEstPose().rotation().degrees()

        if rawGyroReading < 0:
            normalizedGyroReading = 180 - (rawGyroReading + 180)
        elif rawGyroReading > 0:
            normalizedGyroReading = abs(rawGyroReading - 180) + 180
        else:
            normalizedGyroReading = 0.0
        return normalizedGyroReading

    def moveToDesiredAngle(self, desiredGyro):
        normalizedGyro = self.normalizeGyroAngle()
        SPEED_MULTIPLIER = 0.25
        normalizedAndDesiredAngleDifference = normalizedGyro - desiredGyro

        if not ((desiredGyro - 3) < normalizedGyro < (desiredGyro + 3)):
            if 0 < normalizedAndDesiredAngleDifference < 180 or normalizedAndDesiredAngleDifference < -180:
                # CounterClockWise
                return SPEED_MULTIPLIER * MAX_ROTATE_SPEED_RAD_PER_SEC
            else:
                # ClockWise
                return SPEED_MULTIPLIER * -MAX_ROTATE_SPEED_RAD_PER_SEC
        else:
            return 0

    def update(self):
        # value of contoller buttons


        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            vXJoyRaw = self.ctrl.getLeftY() * -1
            vYJoyRaw = self.ctrl.getLeftX() * -1
            vRotJoyRaw = self.ctrl.getRightX() * -1

            # Correct for alliance
            if onRed():
                vXJoyRaw *= -1.0
                vYJoyRaw *= -1.0

            # deadband
            vXJoyWithDeadband = applyDeadband(vXJoyRaw, 0.15)
            vYJoyWithDeadband = applyDeadband(vYJoyRaw, 0.15)
            vRotJoyWithDeadband = applyDeadband(vRotJoyRaw, 0.2)

            # TODO - if the driver wants a slow or sprint button, add it here.
            slowMult = 1.0 if (self.ctrl.getRightBumper()) else 0.75
            #slowMult = 1.0

            # Shape velocity command
            velCmdXRaw = vXJoyWithDeadband * MAX_STRAFE_SPEED_MPS * slowMult
            velCmdYRaw = vYJoyWithDeadband * MAX_FWD_REV_SPEED_MPS * slowMult
            velCmdRotRaw = vRotJoyWithDeadband * MAX_ROTATE_SPEED_RAD_PER_SEC



            # Slew rate limiter
            self.velXCmd = self.velXSlewRateLimiter.calculate(velCmdXRaw)
            self.velYCmd = self.velYSlewRateLimiter.calculate(velCmdYRaw)
            self.velTCmd = self.velTSlewRateLimiter.calculate(velCmdRotRaw)


            #self.gyroResetCmd = self.ctrl.getAButton()
            #self.autoDriveToSpeaker = self.ctrl.getBButton()
            #self.autoDriveToPickup = self.ctrl.getXButton()
            #self.createDebugObstacle = self.ctrl.getYButtonPressed()
            self.connectedFault.setNoFault()

            #------------------------------------------------------------------------------------------------------
            #Noah's Work :D

            if self.ctrl.getBButton():
                self.velTCmd = self.moveToDesiredAngle(0)
                print(self.degreesTillDesiredGyro(0))
            if self.ctrl.getAButton():
                self.velTCmd = self.moveToDesiredAngle(90)
                print(self.degreesTillDesiredGyro(90))
            if self.ctrl.getXButton():
                self.velTCmd = self.moveToDesiredAngle(180)
                print(self.degreesTillDesiredGyro(180))
            if self.ctrl.getYButton():
                # self.velTCmd = self.moveToDesiredAngle(270)
                # print(self.degreesTillDesiredGyro(270))
                pass



            #----------------------------------------------------------------------------------------------------------------



        else:
        # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.velXCmd = 0.0
            self.velYCmd = 0.0
            self.velTCmd = 0.0
            self.gyroResetCmd = False
            self.autoDriveToSpeaker = False
            self.autoDriveToPickup = False
            self.createDebugObstacle = False
            self.connectedFault.setFaulted()




    def getCmd(self) -> DrivetrainCommand:
        retval = DrivetrainCommand()
        retval.velX = self.velXCmd
        retval.velY = self.velYCmd
        retval.velT = self.velTCmd
        return retval

    def getNavToSpeaker(self) -> bool:
        return self.autoDriveToSpeaker
    
    def getNavToPickup(self) -> bool:
        return self.autoDriveToPickup

    def getGyroResetCmd(self) -> bool:
        return self.gyroResetCmd

    def getCreateObstacle(self) -> bool:
        return self.createDebugObstacle