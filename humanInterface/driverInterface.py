from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS,\
MAX_ROTATE_SPEED_RAD_PER_SEC,MAX_TRANSLATE_ACCEL_MPS2,MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
from utils.signalLogging import addLog
from drivetrain.drivetrainControl import DrivetrainControl
#from utils.signalLogging import addLog


class DriverInterface:
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
        self.autoDrive = False
        self.autoSteer = False
        self.createDebugObstacle = False

        # Utility - reset to zero-angle at the current pose
        self.gyroResetCmd = False
        #utility - use robot-relative commands
        self.robotRelative = False
		
        self.processedStrafe = 0
        self.processedRotate = 0
        self.tempStdDevX = 0
        self.tempStdDevY = 0
        self.tempStdDevT = 0
        self.allXMeasures = []
        self.allYMeasures = []
        self.allTMeasures = []
        self.drivetrainCtrl = DrivetrainControl()
        self.sd_record = 0

        addLog("ytest_speed_strafe_level", lambda: self.processedStrafe, "")
        addLog("ytest_speed_rotate_level", lambda: self.processedRotate, "")
        addLog("ytest_standard_dev_x", lambda: self.tempStdDevX, "")
        addLog("ytest_standard_dev_y", lambda: self.tempStdDevY, "")
        addLog("ytest_standard_dev_t", lambda: self.tempStdDevT, "")

        # Logging
        #addLog("DI FwdRev Cmd", lambda: self.velXCmd, "mps")
        #addLog("DI Strafe Cmd", lambda: self.velYCmd, "mps")
        #addLog("DI Rot Cmd", lambda: self.velTCmd, "radps")
        #addLog("DI gyroResetCmd", lambda: self.gyroResetCmd, "bool")
        #addLog("DI autoDriveToSpeaker", lambda: self.autoDriveToSpeaker, "bool")
        #addLog("DI autoDriveToPickup", lambda: self.autoDriveToPickup, "bool")

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

            # TODO - if the driver wants a slow or sprint button, add it here.
            slowMult = 1.0 if (self.ctrl.getRightBumper()) else 0.5

            # Shape velocity command
            velCmdXRaw = vXJoyWithDeadband * MAX_STRAFE_SPEED_MPS * slowMult
            velCmdYRaw = vYJoyWithDeadband * MAX_FWD_REV_SPEED_MPS * slowMult
            velCmdRotRaw = vRotJoyWithDeadband * MAX_ROTATE_SPEED_RAD_PER_SEC * 0.8

            if self.robotRelative:
                velCmdXRaw *= .5
                velCmdYRaw *= .5
                velCmdRotRaw *= .5

            # Slew rate limiter
            self.velXCmd = self.velXSlewRateLimiter.calculate(velCmdXRaw)
            self.velYCmd = self.velYSlewRateLimiter.calculate(velCmdYRaw)
            self.velTCmd = self.velTSlewRateLimiter.calculate(velCmdRotRaw)

            self.gyroResetCmd = self.ctrl.getAButton()

            self.autoDrive = self.ctrl.getBButton()
            self.autoSteer = self.ctrl.getXButton()
            self.createDebugObstacle = self.ctrl.getYButtonPressed()

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

            #locations would go here...

            if what>0:
                posEst = self.drivetrainCtrl.poseEst.posEstLogs[3]
                if self.sd_record==0: #turning on
                    self.allXMeasures = []
                    self.allYMeasures = []
                    self.allTMeasures = []

                og_x = 3.65760732 #we're using sig figs for this hahaha
                og_y = 4.02590805
                rate_x = 0.918765625
                real_x = og_x - (rate_x * what) #order of ops but i don't trust python
                real_y = og_y
                real_t = 0

                if posEst.x_value != 0:
                    self.allXMeasures.append(pow(posEst.x_value-real_x, 2))
                if posEst.y_value != 0:
                    self.allYMeasures.append(pow(posEst.y_value-real_y, 2))
                if posEst.t_value != 0:
                    self.allTMeasures.append(pow(posEst.t_value-real_t, 2))

                self.sd_record = self.sd_record + 1
            else:
                if self.sd_record>0: #turning off
                    self.tempStdDevX = pow(sum(self.allXMeasures)/(len(self.allXMeasures)-1), 0.5)
                    self.tempStdDevY = pow(sum(self.allYMeasures)/(len(self.allYMeasures)-1), 0.5)
                    self.tempStdDevT = pow(sum(self.allTMeasures)/(len(self.allTMeasures)-1), 0.5)
                    # sd (sample) = sqrt(sum((x-m)^2)/(n-1))

                self.sd_record = 0



        else:
            # If the joystick is unplugged, pick safe-state commands and raise a fault
            self.velXCmd = 0.0
            self.velYCmd = 0.0
            self.velTCmd = 0.0
            self.gyroResetCmd = False
            self.autoDrive = False
            self.robotRelative = False
            self.createDebugObstacle = False
            self.connectedFault.setFaulted()




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

    def getAutoDrive(self) -> bool:
        return self.autoDrive

    def getAutoSteer(self) -> bool:
        return self.autoSteer

    def getGyroResetCmd(self) -> bool:
        return self.gyroResetCmd

    def getCreateObstacle(self) -> bool:
        return self.createDebugObstacle

    def getRobotRelative(self):
        return self.robotRelative

