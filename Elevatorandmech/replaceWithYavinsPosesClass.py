from wpimath.geometry import Pose2d
from drivetrain.drivetrainCommand import DrivetrainCommand
from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.ArmCommand import ArmCommand

from humanInterface.operatorInterface import OperatorInterface
class YavinsPoseClassNoChange():
    def __init__(self, arm, driveTrain, elevator):
        self.arm = arm
        self.driveTrain = driveTrain
        self.elevator = elevator

    # Every frame the update will be called, and then later, the updates for the Drivetrain, Elevator, and Arm will be called and they will all "get" their commands.
    def update(self):
        pass

    def getDriveTrainCommand(self, curCommand: DrivetrainCommand):
        return curCommand # if we have nothing to change, we return the current command

    def getElevatorCommand(self, curCommand: ElevatorCommand):
        return curCommand # if we have nothing to change, we return the current command

    def getArmCommand(self, curCommand: ArmCommand):
        return curCommand # if we have nothing to change, we return the current command



class YavinsPoseClassPositionControl():

    def __init__(self, arm, driveTrain, elevator):
        self.arm = arm
        self.driveTrain = driveTrain
        self.elevator = elevator
        self.oInt = OperatorInterface()

    # Every frame the update will be called, and then later, the updates for the Drivetrain, Elevator, and Arm will be called and they will all "get" their commands.
    def update(self):
        pass

    def getDriveTrainCommand(self, curCommand: DrivetrainCommand):
        return curCommand # if we have nothing to change, we return the current command

    def getElevatorCommand(self, curCommand: ElevatorCommand):
        elevatorCommand = ElevatorCommand(
            heightIn=self.oInt.getDesElevatorPosIn(),
            velocityInps=0.0
        )
        return elevatorCommand

    # TODO change this so the arm is a position control loop
    def getArmCommand(self, curCommand: ArmCommand):
        return curCommand # if we have nothing to change, we return the current command
