
from drivetrain.drivetrainCommand import DrivetrainCommand
from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.ArmCommand import ArmCommand
from wpimath.geometry import Pose2d, Rotation2d, Twist2d

from humanInterface.operatorInterface import OperatorInterface
class YavinsPoseClassNoChange():
    def __init__(self, arm, driveTrain, elevator, oInt):
        self.arm = arm
        self.driveTrain = driveTrain
        self.elevator = elevator
        self.oInt = oInt

    # Every frame the update will be called, and then later, the updates for the Drivetrain, Elevator, and Arm will be called and they will all "get" their commands.
    def update(self):
        pass

    def getDriveTrainCommand(self, curCommand: DrivetrainCommand):
        return curCommand # if we have nothing to change, we return the current command

    def getElevatorCommand(self, curCommand: ElevatorCommand):
        return curCommand # if we have nothing to change, we return the current command

    def getArmCommand(self, curCommand: ArmCommand):
        return curCommand # if we have nothing to change, we return the current command

    def deactivate(self):
        pass



class YavinsPoseClassPositionControl():

    def __init__(self, arm, driveTrain, elevator, oInt):
        self.arm = arm
        self.driveTrain = driveTrain
        self.elevator = elevator

    # Every frame the update will be called, and then later, the updates for the Drivetrain, Elevator, and Arm will be called and they will all "get" their commands.
    def update(self):
        pass


    ##### FIX THESE!!!!!! RIGHT NOW THEY AREN'T IMMUNE TO NONE!!!!

    def getDriveTrainCommand(self, curCommand: DrivetrainCommand):
        drivetrainCommand = DrivetrainCommand(
            velX=0.0,
            velY=0.0,
            velT=0.0,
            desPose=Pose2d(0,0,0)
        )
        return curCommand # if we have nothing to change, we return the current command

    def getElevatorCommand(self, curCommand: ElevatorCommand):
        elevatorCommand = ElevatorCommand(
            heightIn=0.0,
            velocityInps=0.0
        )
        return elevatorCommand # if we have nothing to change, we return the current command

    # TODO change this so the arm is a position control loop
    def getArmCommand(self, curCommand: ArmCommand):
        armCommand = ArmCommand(
            angleDeg=0,
            velocityDegps=0.0
        )
        return curCommand # if we have nothing to change, we return the current command

    def deactivate(self):
        pass


class YavinsPoseClassVelocityControl():

    def __init__(self, arm, driveTrain, elevator, oInt):
        self.arm = arm
        self.driveTrain = driveTrain
        self.elevator = elevator
        self.elevatorVelocityInps = 0
        self.armVelocityDegps = 0
        self.oInt = oInt

    # Every frame the update will be called, and then later, the updates for the Drivetrain, Elevator, and Arm will be called and they will all "get" their commands.
    def update(self):
        self.elevatorVelocityInps = self.oInt.elevatorVelYCmd * 60 #was 10
        #print(f"elevatorVelocityInps={self.elevatorVelocityInps}")
        self.armVelocityDegps = self.oInt.armVelYCmd * 720  # was 10


    ##### FIX THESE!!!!!! RIGHT NOW THEY AREN'T IMMUNE TO NONE!!!!

    def getDriveTrainCommand(self, curCommand: DrivetrainCommand):
        return curCommand # if we have nothing to change, we return the current command

    def getElevatorCommand(self, curCommand: ElevatorCommand):
        elevatorCommand = ElevatorCommand(
            heightIn=None,
            velocityInps=self.elevatorVelocityInps
        )
        #print(f"velocityControl={elevatorCommand.heightIn} {elevatorCommand.velocityInps}")
        return elevatorCommand # if we have nothing to change, we return the current command

    # TODO change this so the arm is a position control loop
    def getArmCommand(self, curCommand: ArmCommand):
        armCommand = ArmCommand(
            angleDeg=None,
            velocityDegps=self.armVelocityDegps
        )
        return armCommand # if we have nothing to change, we return the current command

    def deactivate(self):
        pass
