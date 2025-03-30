from positionSchemes.teleop_schemes.defaultPosers import PoserNoChangeDriver, PoserNoChangeOperator
from humanInterface.operatorInterface import ElevArmCmdState

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



    def initialize(self, pdD, pdO, dInt, oInt, drivetTrain, arm, elevator, auto):
        self.pdD = pdD
        self.pdO = pdO
        self.dInt = dInt
        self.oInt = oInt
        self.driveTrain = drivetTrain
        self.arm = arm
        self.elevator = elevator
        self.auto = auto
        self.controllerStateDriver = ElevArmCmdState.UNINITIALIZED
        self.prevControllerStateDriver = self.controllerStateDriver
        self.controllerStateOperator = ElevArmCmdState.UNINITIALIZED
        self.prevControllerStateOperator = self.controllerStateDriver
        self.currentPositionSchemeDriver = PoserNoChangeDriver(self)
        self.currentPositionSchemeOperator = PoserNoChangeOperator(self)
