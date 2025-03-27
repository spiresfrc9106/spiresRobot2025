

from Elevatorandmech.replaceWithYavinsPosesClass import YavinsPoseClassNoChangeOperator, YavinsPoseClassVelocityControlOperator
from Elevatorandmech.RobotPoserCommon import PoseDirectorCommon
from humanInterface.operatorInterface import OperatorInterface, ElevArmCmdState, ReefLeftOrRight
from humanInterface.driverInterface import DriverInterface
from positionSchemes.plunge_v1 import PlungeV1
from positionSchemes.pickup_v1 import PickupV1
from positionSchemes.place_L4_v6_o import PlaceL4V6O
from positionSchemes.place_L3_v1 import PlaceL3V1
from positionSchemes.place_L2_v1 import PlaceL2V1
from positionSchemes.place_L1_v1 import PlaceL1V1
from utils.signalLogging import addLog
from utils.singleton import Singleton

# add a state variable that keeps track of if one of the left3 dpads where pressed or one of the right3 dpads where press
# default to the left3
# add a method that returns the state variable
# pass the poseDirectorOperator singleton to all calls that create posers
# add a method to operator interface that keeps track of if the left3 buttons on the dpad are pressed or the right3
# in posedirector update make the state variable updated.

class PoseDirectorOperator(metaclass=Singleton):

    def __init__(self):
        self.common = PoseDirectorCommon()

    def initialize(self):


        self.common.controllerStateOperator = ElevArmCmdState.UNINITIALIZED
        self.common.prevControllerStateOperator = self.common.controllerStateOperator
        self.common.currentPositionSchemeOperator = YavinsPoseClassNoChangeOperator(self.common.arm, self.common.driveTrain, self.common.elevator, self.common.oInt)
        self.getElevatorCommand = lambda curCommand :  self.common.currentPositionSchemeOperator.getElevatorCommand(curCommand)
        self.getArmCommand = lambda curCommand : self.common.currentPositionSchemeOperator.getArmCommand(curCommand)
        self.schemeProg = 0
        self.dashboardState = 1 # State 1, put the autonomous menu back up on the webserver dashboard
        addLog("RP/schemeProg", lambda: self.schemeProg, "") # don't delete this.
        addLog("RP/dashboardState", lambda: self.dashboardState, "") # don't delete this.
        # addLog("RP/controllerState", lambda: self.common.controllerStateOperator, "int")

    def setDashboardState(self, dashboardState: int):
        self.dashboardState = dashboardState

    def update(self, isAuton=False):

        if (not isAuton) and self._isControllerStateChanging():
            self.common.currentPositionSchemeOperator.deactivate()
            self.common.currentPositionSchemeOperator = self.pickTheNewScheme()
            self.getElevatorCommand = lambda curCommand: self.common.currentPositionSchemeOperator.getElevatorCommand(curCommand)
            self.getArmCommand = lambda curCommand : self.common.currentPositionSchemeOperator.getArmCommand(curCommand)
            # self.progress = self.common.currentPositionSchemeOperator.schemeProg *100
        self.common.currentPositionSchemeOperator.update()
        if hasattr(self.common.currentPositionSchemeOperator, "schemeProg"):
            self.schemeProg=round(self.common.currentPositionSchemeOperator.schemeProg*100)
            # xyzzy, todo ask Yavin, shouldn't we set a dashboard state here?
        else:
            self.setDashboardState(3)
        if isAuton:
            self.setDashboardState(2)

    def _isControllerStateChanging(self)->bool:
        nextState = self.common.oInt.getElevArmCmdState()
        changed = False
        if nextState != self.common.controllerStateOperator:
            print(
                f"RobotPoser: state changing from {self.common.controllerStateOperator.name}({self.common.controllerStateOperator}) to {nextState.name} ({nextState})")
            self.common.prevControllerStateOperator = self.common.controllerStateOperator
            self.common.controllerStateOperator = nextState
            changed = True
        return changed

    def pickTheNewScheme(self)->None:
        match self.common.controllerStateOperator:
            case ElevArmCmdState.UNINITIALIZED:
                return YavinsPoseClassNoChangeOperator(self.common.arm, self.common.driveTrain, self.common.elevator, self.common.oInt)
            case ElevArmCmdState.VEL_CONTROL:
                return YavinsPoseClassVelocityControlOperator(self.common.arm, self.common.driveTrain, self.common.elevator, self.common.oInt) # todo fix me

            case ElevArmCmdState.PLUNGE:
                self.setDashboardState(5)
                return PlungeV1(self.common.arm, self.common.driveTrain, self.common.elevator, self.common.oInt) # todo fix me
            case ElevArmCmdState.RECEIVE_CORAL:
                self.setDashboardState(4)
                return PickupV1(self.common.arm, self.common.driveTrain, self.common.elevator, self.common.oInt) # todo fix me
            case ElevArmCmdState.L1:
                self.setDashboardState(6)
                return PlaceL1V1(self.common.arm, self.common.driveTrain, self.common.elevator, self.common.oInt)  # todo fix me
            case ElevArmCmdState.L2:
                self.setDashboardState(6)
                return PlaceL2V1(self.common.arm, self.common.driveTrain, self.common.elevator, self.common.oInt)  # todo fix me
            case ElevArmCmdState.L3:
                self.setDashboardState(6)
                return PlaceL3V1(self.common.arm, self.common.driveTrain, self.common.elevator, self.common.oInt)  # todo fix me
            case ElevArmCmdState.L4:
                self.setDashboardState(6)
                return PlaceL4V6O(self.common)  # todo fix me
            case _:
                return YavinsPoseClassNoChangeOperator(self.common.arm, self.common.driveTrain, self.common.elevator, self.common.oInt)  # todo fix me




