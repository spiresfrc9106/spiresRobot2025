import math

import wpilib
from wpimath.geometry import Pose2d
from utils.fieldTagLayout import FieldTagLayout
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.controlStrategies.trajectoryGuts import TrajectoryGuts
from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.ArmCommand import ArmCommand
from wpimath.geometry import Pose2d
from wpilib import Timer
from drivetrain.controlStrategies.trajectoryGuts import ChoreoTrajectoryState
from utils.units import deg2Rad, rad2Deg, in2m, m2in

### these are intrisic to any pos scheme class


class SetupScheme:
    def __init__(self, arm, base, elev):
        self.startTime = Timer.getFPGATimestamp()
        self.setupBase = base
        self.setupArm = arm
        self.setupElev = elev
        self.changeInTime = 0
        self.waitTimes = {}
        self.schemeProg = 0
        self.localProg = 0
        self.setDriveTrainBaseCommand(None)
        self.armCmd = None
        self.elevCmd = None
        self.basePrimitiveCmd = None
        self.bestTag = Pose2d()
        self.initMaxDistLoc = 100
        self.initMaxDistDeg = 100
        self.initMaxDistLen = 100
        self.initLoc = 0
        self.initDeg = 0
        self.initLen = 0

    def nextState(self):
        self.currentState = self.currentState + 1
        self.localProg = 0

    def setBase(self, base):
        self.setupBase = base

    def isSim(self):
        return wpilib.RobotBase.isSimulation()

    def completedAwait(self, waitName, duration):
        now = Timer.getFPGATimestamp()
        if self.waitTimes.get(waitName, -1) == -1:
            self.waitTimes[waitName] = now
        if now - self.waitTimes[waitName] >= duration:
            return True
        else:
            return False

    def updateProgressTrajectory(self):
        if self.initLoc != self.bestTag:
            self.initMaxDistLoc = self.getFullDistanceSubjective()
        self.initLoc = self.bestTag
        current = self.getFullDistanceSubjective()
        progress = (self.initMaxDistLoc-current)/self.initMaxDistLoc
        self.localProg = max(progress, self.localProg)

    def updateProgressArm(self):
        full_delta = abs(self.armCmd[0] - self.setupArm.getPosition())
        if self.initDeg != self.armCmd[0]:
            self.initMaxDistDeg = full_delta
        self.initDeg = self.armCmd[0]
        progress = (self.initMaxDistDeg-full_delta)/self.initMaxDistDeg
        self.localProg = max(progress, self.localProg)

    def updateProgressElev(self):
        full_delta = abs(self.elevCmd[0] - self.setupElev.getPosition())
        if self.initLen != self.elevCmd[0]:
            self.initMaxDistLen = full_delta
        self.initLen = self.elevCmd[0]
        progress = (self.initMaxDistLen - full_delta) / self.initMaxDistLen
        self.localProg = max(progress, self.localProg)

    def getFullDistanceSubjective(self):
        desPose = self.bestTag
        curPose = self.setupBase.tcPoseEst.getCurEstPose()
        desX = YPose(desPose).x
        desY = YPose(desPose).y
        desT = YPose(desPose).t
        curX = YPose(curPose).x
        curY = YPose(curPose).y
        curT = YPose(curPose).t
        dist_translate = math.sqrt(pow((desX-curX), 2)+pow((desY-curY), 2))
        dist_rotate = abs(curT-desT)
        return dist_translate+dist_rotate

    def completedTrajectory(self, base, max_in=1, max_deg=1):
        desPose = self.bestTag
        curPose = base.tcPoseEst.getCurEstPose()
        desX = YPose(desPose).x
        desY = YPose(desPose).y
        desT = YPose(desPose).t
        curX = YPose(curPose).x
        curY = YPose(curPose).y
        curT = YPose(curPose).t
        dist_translate = math.sqrt(pow((desX-curX), 2)+pow((desY-curY), 2))
        dist_rotate = abs(curT-desT)
        return dist_translate < in2m(max_in) and dist_rotate < max_deg

    def setArmCommand(self, x_deg: float | None, v_degps: float | None):
        if x_deg is not None:
            if v_degps is not None:
                self.armCmd = (x_deg, v_degps)
            else:
                self.armCmd = (x_deg, 0)
        else:
            if v_degps is not None:
                self.armCmd = (x_deg, v_degps)
            else:
                self.armCmd = None

    def setElevatorCommand(self, x_in: float | None, v_inps: float | None):
        if x_in is not None:
            if v_inps is not None:
                self.armCmd = (x_in, v_inps)
            else:
                self.armCmd = (x_in, 0)
        else:
            if v_inps is not None:
                self.armCmd = (x_in, v_inps)
            else:
                self.armCmd = None

    def setDriveTrainBaseCommand(self, pose: Pose2d | None, vxMps: float = 0.0, vyMps: float = 0.0, vtRadps: float = 0.0):

        if pose is None:
            self.baseCmd = None
            self.setupBase.tcTraj.setCmdFromPoser(None)
        else:
            if True:
                # save these off in base command for historical reasons as we refactor, might never be used
                self.baseCmd = (pose,  vxMps, vyMps, vtRadps)

                self.setupBase.tcTraj.setCmdFromPoser(
                    ChoreoTrajectoryState(
                        timestamp=1,  # TODO: no idea if this should be some sort of other type of time...
                        x=pose.X(),
                        y=pose.Y(),
                        heading=pose.rotation().radians(),
                        velocityX=vxMps,
                        velocityY=vyMps,
                        angularVelocity=vtRadps
                    )
                )

    def deactivate(self):
        self.setDriveTrainBaseCommand(None, self.setupBase)



    def getDriveTrainCommand(self, curCommand: DrivetrainCommand):
        drivetrain_cmd = curCommand
        if self.baseCmd is not None:
            drivetrain_cmd = drivetrain_cmd
        elif self.basePrimitiveCmd is not None:
            drivetrain_cmd = DrivetrainCommand(
                velX=self.basePrimitiveCmd[0],
                velY=self.basePrimitiveCmd[1],
                velT=self.basePrimitiveCmd[2],
            )
        return drivetrain_cmd  # if we have nothing to change, we return the current command

    def inchesToMeters(self, inches):
        return inches/39.37

    def getElevatorCommand(self, curCommand: ElevatorCommand):
        elev_cmd = curCommand
        if self.elevCmd is not None:
            elev_cmd = ElevatorCommand(
                heightIn=self.elevCmd[0],
                velocityInps=self.elevCmd[1]
            )
        return elev_cmd  # if we have nothing to change, we return the current command

    def getArmCommand(self, curCommand: ArmCommand):
        arm_cmd = curCommand
        if self.armCmd is not None:
            arm_cmd = ArmCommand(
                angleDeg=self.armCmd[0],
                velocityDegps=self.armCmd[1]
            )
        return arm_cmd  # if we have nothing to change, we return the current command

class ArmConsts:
    def __init__(self):
        ## DEGS
        self.posLow = 1
        self.posMedium = 2
        self.posHigh = 3
        ## DEG/S
        self.velLow = 1
        self.velMedium = 2
        self.velHigh = 3


class ElevConsts:
    def __init__(self):
        ## INCHES
        self.posLow = 1
        self.posLowMid = 1.5  # this is the location that the robot should be right before hitting coral
        self.posMedium = 55 #45  # middle of elevator, default location.
        self.posHigh = 3
        ## INCHES/S
        self.velLow = 1
        self.velMedium = 2
        self.velHigh = 3

class YPose:
    def __init__(self, position: Pose2d):
        self.x = position.X()
        self.y = position.Y()
        self.t = position.rotation().radians()
