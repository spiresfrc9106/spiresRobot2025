import wpilib
from wpimath.geometry import Pose2d
from utils.fieldTagLayout import FieldTagLayout
from drivetrain.drivetrainCommand import DrivetrainCommand
from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.ArmCommand import ArmCommand
from wpimath.geometry import Pose2d
from wpilib import Timer
from drivetrain.controlStrategies.trajectory import ChoreoTrajectoryState

### these are intrisic to any pos scheme class


class SetupScheme:
    def __init__(self, arm, base, elev):
        self.startTime = Timer.getFPGATimestamp()
        self.changeInTime = 0
        self.waitTimes = {}
        self.schemeProg = 0
        self.localProg = 0
        self.baseCmd = None
        self.armCmd = None
        self.elevCmd = None
        self.basePrimitiveCmd = None

    def nextState(self):
        self.currentState = self.currentState + 1
        self.localProg = 0

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

    def completedTrajectory(self, base):
        return abs(base.cmdVelX) < 0.08 and abs(base.cmdVelY) < 0.08 and abs(base.cmdVelT) < 3

    def getDriveTrainCommand(self, curCommand: DrivetrainCommand):
        drivetrain_cmd = curCommand
        if self.baseCmd is not None:
            drivetrain_cmd = ChoreoTrajectoryState(
                timestamp = 1, #TODO: no idea if this should be some sort of other type of time...
                x=YPose(self.baseCmd[0]).x,
                y=YPose(self.baseCmd[0]).y,
                heading = YPose(self.baseCmd[0]).t,
                velocityX=self.baseCmd[1],
                velocityY=self.baseCmd[2],
                angularVelocity=self.baseCmd[3]
            )
        if self.basePrimitiveCmd is not None:
            drivetrain_cmd = DrivetrainCommand(
                velX=self.basePrimitiveCmd[0],
                velY=self.basePrimitiveCmd[1],
                velT=self.basePrimitiveCmd[2],
            )
        return drivetrain_cmd  # if we have nothing to change, we return the current command

    def inchesToMeters(self, inches):
        return inches/39.37

        # self.timestamp = timestamp
        # self.x = x
        # self.y = y
        # self.heading = heading
        # self.velocityX = velocityX
        # self.velocityY = velocityY
        # self.angularVelocity = angularVelocity

    # def getDriveTrainCommand(self, curCommand: DrivetrainCommand):
    #     drivetrain_cmd = curCommand
    #     if self.baseCmd is not None:
    #         drivetrain_cmd = DrivetrainCommand(
    #             velX=self.baseCmd[1],
    #             velY=self.baseCmd[2],
    #             velT=self.baseCmd[3],
    #             desPose=self.baseCmd[0]
    #         )
    #     return drivetrain_cmd  # if we have nothing to change, we return the current command

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
        self.posMedium = 45  # middle of elevator, default location.
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