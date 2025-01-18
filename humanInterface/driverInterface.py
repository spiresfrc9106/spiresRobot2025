from math import ceil
from math import floor

from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS,\
MAX_ROTATE_SPEED_RAD_PER_SEC,MAX_TRANSLATE_ACCEL_MPS2,MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from utils.units import deg2Rad
from drivetrain.drivetrainControl import DrivetrainControl


#from utils.signalLogging import addLog



class DriverInterface:
    """Class to gather input from the driver of the robot"""

    def __init__(self):
        self.poseEst = DrivetrainControl().poseEst
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

    def update(self):
        # value of controller buttons

        if self.ctrl.isConnected():
            """
            rawGyroReading = self.poseEst.getCurEstPose().rotation().degrees()
            goal = None

            #ranges gyro from -180 - +180
            niceGyro -= Math.floor(rawGyroReading / 360 + 0.5) * 360

            # Figures what goal deg is from button pushed
            if self.ctrl.getYButton() and niceGyro != 0:
                goal = 0
            elif self.ctrl.getXButton() and not 268 < niceGyro < 272:
                goal = 270
            elif self.ctrl.getAButton() and niceGyro != 180:
                goal = 180
            elif self.ctrl.getBButton() and niceGyro != 90:
                goal = 90
            # Figures if left or right is faster from cur rawGyroReading and goal
            if goal is None:
                vRotJoyRaw = self.ctrl.getRightX() * -1
            elif abs(niceGyro - (goal + 360)) >= niceGyro - goal:
                vRotJoyRaw = 1
            else:
                vRotJoyRaw = -1
            """
            rawGyroReadingDegs = self.poseEst.getCurEstPose().rotation().degrees()
            goalDegs = None

            if onRed():
                if self.ctrl.getAButton():
                    goalDegs = 0
                elif self.ctrl.getXButton():
                    goalDegs = -90
                elif self.ctrl.getYButton():
                    goalDegs = 179.9
                elif self.ctrl.getBButton():
                    goalDegs = 90
            else:
                if self.ctrl.getYButton():
                    goalDegs = 0
                elif self.ctrl.getBButton():
                    goalDegs = -90
                elif self.ctrl.getAButton():
                    goalDegs = 179.9
                elif self.ctrl.getXButton():
                    goalDegs = 90

            if goalDegs is None:
                vRotJoyRaw = self.ctrl.getRightX() * -1
            else:
                normalizedRawGyroDegs = rawGyroReadingDegs - floor(rawGyroReadingDegs / 360 + 0.5) * 360
                signedDistanceDegs = normalizedRawGyroDegs - goalDegs
                normalizedDistanceDegs = signedDistanceDegs - floor(signedDistanceDegs / 360 + 0.5) * 360

                if normalizedDistanceDegs<0:
                    vRotJoyRaw = 1
                else:
                    vRotJoyRaw = -1

                print(f"goal={goalDegs:07.1f} normalizedRawGyroDegs={normalizedRawGyroDegs:07.1f} normalizedDistanceDegs={normalizedDistanceDegs:07.1f} vRotJoyRaw={vRotJoyRaw:07.1f}")

            # Convert from  joystick sign/axis conventions to robot velocity conventions
            vXJoyRaw = self.ctrl.getLeftY() * -1
            vYJoyRaw = self.ctrl.getLeftX() * -1
            #vRotJoyRaw = self.ctrl.getRightX() * -1

            # Correct for alliance
            if onRed():
                vXJoyRaw *= -1.0
                vYJoyRaw *= -1.0

            # deadband
            vXJoyWithDeadband = applyDeadband(vXJoyRaw, 0.15)
            vYJoyWithDeadband = applyDeadband(vYJoyRaw, 0.15)
            vRotJoyWithDeadband = applyDeadband(vRotJoyRaw, 0.2)

            useProportionalCmd = self.ctrl.getLeftBumper()
            if useProportionalCmd:
                if abs(normalizedDistanceDegs) > 30:
                    rotSlowMult = 1.0
                else:
                    rotSlowMult = abs(normalizedDistanceDegs) / 40
            else:
                rotSlowMult = 0.25 if not (goalDegs is None) else 1.0

            # TODO - if the driver wants a slow or sprint button, add it here.
            slowMult = 1.0 if (self.ctrl.getRightBumper()) else 0.75
            #slowMult = 1.0

            # Shape velocity command
            velCmdXRaw = vXJoyWithDeadband * MAX_STRAFE_SPEED_MPS * slowMult
            velCmdYRaw = vYJoyWithDeadband * MAX_FWD_REV_SPEED_MPS * slowMult
            velCmdRotRaw = vRotJoyWithDeadband * MAX_ROTATE_SPEED_RAD_PER_SEC * rotSlowMult

            # Slew rate limiter
            self.velXCmd = self.velXSlewRateLimiter.calculate(velCmdXRaw)
            self.velYCmd = self.velYSlewRateLimiter.calculate(velCmdYRaw)
            self.velTCmd = self.velTSlewRateLimiter.calculate(velCmdRotRaw)

            #self.gyroResetCmd = self.ctrl.getAButton()

            # self.autoDriveToSpeaker = self.ctrl.getBButton()
            # self.autoDriveToPickup = self.ctrl.getXButton()
            # self.createDebugObstacle = self.ctrl.getYButtonPressed()

            self.connectedFault.setNoFault()

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

    def getNormalizedDistanceDegs(goalDegs: float, rawGyroReadingDegs: float) -> float:
