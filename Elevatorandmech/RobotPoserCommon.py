from Elevatorandmech.replaceWithYavinsPosesClass import YavinsPoseClassNoChangeDriver, YavinsPoseClassNoChangeOperator
from humanInterface.operatorInterface import OperatorInterface, ElevArmCmdState, ReefLeftOrRight
from humanInterface.driverInterface import DriverInterface

from utils.singleton import Singleton

# add a state variable that keeps track of if one of the left3 dpads where pressed or one of the right3 dpads where press
# default to the left3
# add a method that returns the state variable
# pass the poseDirectorOperator singleton to all calls that create posers
# add a method to operator interface that keeps track of if the left3 buttons on the dpad are pressed or the right3
# in posedirector update make the state variable updated.

class PoseDirectorCommon(metaclass=Singleton):
    def __init__(self):
        super().__init__()



    def initialize(self, dInt, oInt, drivetTrain, arm, elevator):
        self.dInt = dInt
        self.oInt = oInt
        self.driveTrain = drivetTrain
        self.arm = arm
        self.elevator = elevator
        self.controllerStateDriver = ElevArmCmdState.UNINITIALIZED
        self.prevControllerStateDriver = self.controllerStateDriver
        self.currentPositionSchemeDriver = YavinsPoseClassNoChangeDriver(self.driveTrain, self.oInt)

        self.controllerStateOperator = ElevArmCmdState.UNINITIALIZED
        self.prevControllerStateOperator = self.controllerStateDriver
        self.currentPositionSchemeOperator = YavinsPoseClassNoChangeOperator(self.arm, self.elevator, self.dInt)
