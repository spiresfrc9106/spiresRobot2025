from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS, \
MAX_ROTATE_SPEED_RAD_PER_SEC, MAX_TRANSLATE_ACCEL_MPS2,MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from utils.signalLogging import addLog
from utils.singleton import Singleton

HAS_DRIVETRAIN = False # xyzzy TODO talk to Yavin about this

class DriverInterface(metaclass=Singleton):
    """Class to gather input from the driver of the robot"""

    def __init__(self):
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
        #utility - use robot-relative commands
        self.robotRelative = False
        self.motorTestCmd = 0

        self.processedStrafe = 0
        self.processedRotate = 0
        self.tempStdDevX = 0
        self.tempStdDevY = 0
        self.tempStdDevT = 0
        self.allXMeasures = []
        self.allYMeasures = []
        self.allTMeasures = []
        if HAS_DRIVETRAIN:
            pass
        self.sd_record = 0

        addLog("ytest_speed_strafe_level", lambda: self.processedStrafe, "")
        addLog("ytest_speed_rotate_level", lambda: self.processedRotate, "")

        # Logging
        addLog("DI/FwdRev Cmd", lambda: self.velXCmd, "mps")
        addLog("DI/Strafe Cmd", lambda: self.velYCmd, "mps")
        addLog("DI/Rot Cmd", lambda: self.velTCmd, "radps")
        addLog("DI/gyroResetCmd", lambda: self.gyroResetCmd, "bool")
        addLog("DI/autoDriveToSpeaker", lambda: self.autoDriveToSpeaker, "bool")
        addLog("DI/autoDriveToPickup", lambda: self.autoDriveToPickup, "bool")
        addLog("DI/motorTestCmd", lambda: self.motorTestCmd, "frac")

    def update(self):
        # value of contoller buttons

        if self.ctrl.isConnected():
            # Convert from  joystic sign/axis conventions to robot velocity conventions
            vXJoyRaw = self.ctrl.getLeftY() * -1
            vYJoyRaw = self.ctrl.getLeftX() * -1
            vRotJoyRaw = self.ctrl.getRightX() * -1


            self.robotRelative = self.ctrl.getLeftBumper()

            if not self.robotRelative:
                # Correct for alliance
                if onRed():
                    vXJoyRaw *= -1.0
                    vYJoyRaw *= -1.0

            # deadband
            vXJoyWithDeadband = applyDeadband(vXJoyRaw, 0.05)
            vYJoyWithDeadband = applyDeadband(vYJoyRaw, 0.05)
            vRotJoyWithDeadband = applyDeadband(vRotJoyRaw, 0.05)

            slowMult = 1.0 if (self.ctrl.getRightBumper()) else 0.25

            # Shape velocity command
            velCmdXRaw = vXJoyWithDeadband * MAX_STRAFE_SPEED_MPS * slowMult
            velCmdYRaw = vYJoyWithDeadband * MAX_FWD_REV_SPEED_MPS * slowMult
            velCmdRotRaw = vRotJoyWithDeadband * MAX_ROTATE_SPEED_RAD_PER_SEC * slowMult


            if self.robotRelative:
                velCmdXRaw *= .5
                velCmdYRaw *= .5
                velCmdRotRaw *= .5


            # Slew rate limiter
            self.velXCmd = self.velXSlewRateLimiter.calculate(velCmdXRaw)
            self.velYCmd = self.velYSlewRateLimiter.calculate(velCmdYRaw)
            self.velTCmd = self.velTSlewRateLimiter.calculate(velCmdRotRaw)

            self.gyroResetCmd = False
            self.autoDriveToSpeaker = False
            self.autoDriveToPickup = False
            self.createDebugObstacle = False

            self.motorTestCmd = self.ctrl.getLeftTriggerAxis() - self.ctrl.getRightTriggerAxis()

            self.connectedFault.setNoFault()

            what = 0
            if self.ctrl.getXButton():
                what = 4
            if self.ctrl.getAButton():
                what = 3
            if self.ctrl.getYButton():
                what = 2
            if self.ctrl.getBButton():
                what = 1


            self.sd_record = 0

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



    def getMotorTestPowerRpm(self):
        rpms = 2000
        return self.motorTestCmd * rpms

    def getCmd(self) -> DrivetrainCommand:
        retval = DrivetrainCommand()
        retval.velX = self.velXCmd
        retval.velY = self.velYCmd
        retval.velT = self.velTCmd
        ####TESTING PURPOSES
        diagStrafe = pow(pow(self.velXCmd, 2) + pow(self.velYCmd, 2),0.5)
        self.processedStrafe = abs(round(diagStrafe/MAX_FWD_REV_SPEED_MPS*10))
        self.processedRotate = abs(round(self.velTCmd/MAX_ROTATE_SPEED_RAD_PER_SEC*10))

        return retval

    def getVelXCmd(self):
        return self.velXCmd

    def getVelYCmd(self):
        return self.velYCmd

    def getVelTCmd(self):
        return self.velTCmd

    def getNavToSpeaker(self) -> bool:
        return self.autoDriveToSpeaker
    
    def getNavToPickup(self) -> bool:
        return self.autoDriveToPickup

    def getGyroResetCmd(self) -> bool:
        return self.gyroResetCmd

    def getCreateObstacle(self) -> bool:
        return self.createDebugObstacle

    def getRobotRelative(self):
        return self.robotRelative
