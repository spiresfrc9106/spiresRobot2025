from drivetrain.drivetrainCommand import DrivetrainCommand
from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.ArmCommand import ArmCommand
from wpimath.geometry import Pose2d
from wpilib import Timer


### these are intrisic to any pos scheme class


class TemplateScheme:

    def __init__(self):
        pass

    def nextState(self):
        self.currentState = self.currentState + 1

    def completedAwait(self, waitName, duration):
        now = Timer.getFPGATimestamp()
        if self.waitTimes.get(waitName, -1) == -1:
            self.waitTimes[waitName] = now
        if now - self.waitTimes[waitName] >= duration:
            return True
        else:
            return False

    def getDriveTrainCommand(self, curCommand: DrivetrainCommand):
        drivetrain_cmd = curCommand
        if self.baseCmd is not None:
            drivetrain_cmd = DrivetrainCommand(
                velX=self.baseCmd[1],
                velY=self.baseCmd[2],
                velT=self.baseCmd[3],
                desPose=Pose2d(self.baseCmd[0])
            )
        return drivetrain_cmd  # if we have nothing to change, we return the current command

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
    def _init_(self):
        ## DEGS
        self.posLow = 1
        self.posMedium = 2
        self.posHigh = 3
        ## DEG/S
        self.velLow = 1
        self.velMedium = 2
        self.velHigh = 3


class ElevConsts:
    def _init_(self):
        ## INCHES
        self.posLow = 1
        self.posLowMid = 1.5  # this is the location that the robot should be right before hitting coral
        self.posMedium = 2  # middle of elevator, default location.
        ## INCHES/S
        self.posHigh = 3
        self.velLow = 1
        self.velMedium = 2
        self.velHigh = 3