from wpimath import applyDeadband
from wpimath.filter import SlewRateLimiter
from wpilib import XboxController
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import MAX_FWD_REV_SPEED_MPS,MAX_STRAFE_SPEED_MPS,\
MAX_ROTATE_SPEED_RAD_PER_SEC,MAX_TRANSLATE_ACCEL_MPS2,MAX_ROTATE_ACCEL_RAD_PER_SEC_2
from utils.allianceTransformUtils import onRed
from utils.faults import Fault
#from utils.signalLogging import addLog
from wpilib import Timer
from utils.units import deg2Rad, rad2Deg
from drivetrain.drivetrainControl import DrivetrainControl
import math
from utils.signalLogging import addLog

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

        self.lastIdealDir = None
        self.startRotateTime = None
        self.fullRotateDistance = None

        # Logging
        #addLog("DI FwdRev Cmd", lambda: self.velXCmd, "mps")
        #addLog("DI Strafe Cmd", lambda: self.velYCmd, "mps")
        #addLog("DI Rot Cmd", lambda: self.velTCmd, "radps")
        #addLog("DI gyroResetCmd", lambda: self.gyroResetCmd, "bool")
        #addLog("DI autoDriveToSpeaker", lambda: self.autoDriveToSpeaker, "bool")
        #addLog("DI autoDriveToPickup", lambda: self.autoDriveToPickup, "bool")

        self.v_time = 0.0
        addLog("velTCmd", lambda:self.v_time, "deg")

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
            # slowMult = 1.0

            # Shape velocity command
            velCmdXRaw = vXJoyWithDeadband * MAX_STRAFE_SPEED_MPS * slowMult
            velCmdYRaw = vYJoyWithDeadband * MAX_FWD_REV_SPEED_MPS * slowMult
            velCmdRotRaw = vRotJoyWithDeadband * MAX_ROTATE_SPEED_RAD_PER_SEC

            # Slew rate limiter
            self.velXCmd = self.velXSlewRateLimiter.calculate(velCmdXRaw)
            self.velYCmd = self.velYSlewRateLimiter.calculate(velCmdYRaw)
            self.velTCmd = self.velTSlewRateLimiter.calculate(velCmdRotRaw)



            dirBtnDeg = {
                "Y": 0,
                "B": 90,
                "A": 180,
                "X": 270
            }
            idealReading = None
            if self.ctrl.getAButton():
                idealReading = dirBtnDeg["A"]
            if self.ctrl.getBButton():
                idealReading = dirBtnDeg["B"]
            if self.ctrl.getXButton():
                idealReading = dirBtnDeg["X"]
            if self.ctrl.getYButton():
                idealReading = dirBtnDeg["Y"]

            #CHANGE THE GYRO READING TO REAL VALUE, and CHANGE THE FULL CIRCLE to RAD/DEG depending.
            fullCircle = 360
            rawGyroReading = self.poseEst.getCurEstPose().rotation().degrees()  #returns 180 (L) to -180 (R)
            #######print(rawGyroReading)
            halfCircle = fullCircle/2
            if rawGyroReading<0: #right
                rawGyroReading = abs(rawGyroReading)
                #rawGyroReading = 360-(abs(rawGyroReading)%360)
            else:
                rawGyroReading = 360 - (abs(rawGyroReading) % 360)
            gyroReading = rawGyroReading % 360

            #######print(gyroReading)

            if (idealReading is not None) and abs(idealReading-gyroReading)>fullCircle/(360*2):
                if idealReading != self.lastIdealDir:
                    #this is the first execution
                    self.startRotateTime = Timer.getFPGATimestamp()
                    self.fullRotateDistance = gyroReading-idealReading
                    self.lastRotSpeed = 0
                    print("new path began")
                    ###print(f'time: {((min(abs(self.fullRotateDistance), fullCircle-abs(self.fullRotateDistance)))/halfCircle)}')

                pathDistance = min(abs(self.fullRotateDistance), fullCircle-abs(self.fullRotateDistance))
                currRotateDistance = gyroReading-idealReading
                distanceLeft = min(abs(currRotateDistance), fullCircle - abs(currRotateDistance))
                completedDist = pathDistance - distanceLeft

                t = pathDistance/halfCircle
                idealSpeed = (distanceLeft/t)+50




                # if ((idealSpeed-self.lastRotSpeed)>self.lastRotSpeed*0.5):
                #     desiredSpeed = (self.lastRotSpeed + desiredSpeed)/2

                if (idealSpeed-self.lastRotSpeed)>45:
                    desiredSpeed = self.lastRotSpeed + 45

                max_acc = 45
                acc_diff = idealSpeed-self.lastRotSpeed
                acc_diff_abs = abs(acc_diff)
                sign = acc_diff/acc_diff_abs
                selected_acc = min(acc_diff_abs,max_acc) * sign

                v_time = self.lastRotSpeed + selected_acc

                if distanceLeft<3:
                    v_time = max(distanceLeft*4,0)

                t_time = Timer.getFPGATimestamp()-self.startRotateTime
                print(f'time={t_time}')

                self.lastRotSpeed = v_time
                self.v_time = v_time



                #build a customizable function:
                #x = a*b/3*pow(t*c,3)+i*t
                #a = (x-i*t)/pow(t*c,3)*3/b
                #v = a*b*pow(c,3)*pow(t,2)+i //derived
                # xmax = pathDistance
                # tmax = pathDistance/halfCircle
                # i = halfCircle/4 #init velocity, 45deg/50 which is significant change bt each exec (per 20ms)
                # b = 1
                # c = 1
                # a = (xmax - i * tmax) / pow(tmax * c, 3) * 3 / b
                # #print(f'a value: {a}')
                #
                # #begin crazy computations
                # #-----------------------
                # #time (t) based
                # t_time = Timer.getFPGATimestamp()-self.startRotateTime
                # v_time = a * b * pow(c, 3) * pow(t_time, 2) + i
                # print(f'{a} * {b} * {c}^3 * t^2 + {i}')
                # ###print(f'time since: {t_time}')
                # #print(v_time)
                #-----------------------
                #performance (x) based
                # x_ideal = a * b / 3 * pow(t_time * c, 3) + i * t_time
                # x_current=completedDist
                ###print(f'gryo: {gyroReading}')
                ###print(f'distance completed: {completedDist} of {self.fullRotateDistance}')
                # if abs((x_ideal - x_current))>(halfCircle/180*10):
                #     ###print("calculating")
                #     x=0
                #     t=0
                #     n=0
                #     while x<x_current and t<tmax and n<50:
                #         x = a * b / 3 * pow(t * c, 3) + i * t
                #         t += 0.02
                #         n += 1
                #     t_performance = t
                #     v_performance = a * b * pow(c, 3) * pow(t_performance, 2) + i
                # else:
                #     v_performance = 0
                v_performance = 0
                #-----------------------
                v_selected = min(max(v_performance, v_time),(MAX_ROTATE_SPEED_RAD_PER_SEC/(3.142*2)*fullCircle))


                # if (abs(idealReading-gyroReading)<fullCircle/360*5):
                #     v_selected = 0

                #debugging fun stuff
                # if v_selected == v_performance and v_selected == v_time:
                #     print("pos&time directed same v")
                # elif v_selected==v_time:
                #     print("time-based v selected")
                # elif v_selected==v_performance:
                #     print("pos-based v selected")
                # else:
                #     print("v max reached")

                direction = 1.0
                if self.fullRotateDistance>halfCircle:
                    direction = -1.0
                self.velTCmd = deg2Rad(direction * v_selected)
            else:
                pass

            self.lastIdealDir = idealReading

            ######IMPORTANT: it is vital that these are like followed so the original code cant control stuff
            self.gyroResetCmd = False
            self.autoDriveToSpeaker = False
            self.autoDriveToPickup = False
            self.createDebugObstacle = False

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