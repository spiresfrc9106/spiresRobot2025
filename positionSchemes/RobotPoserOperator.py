from positionSchemes.teleop_schemes.defaultPosers import PoserNoChangeOperator, PoserVelocityControlOperator
from positionSchemes.RobotPoserCommon import PoseDirectorCommon
from humanInterface.operatorInterface import ElevArmCmdState
from positionSchemes.teleop_schemes.plunge_v1 import PlungeV1
from positionSchemes.teleop_schemes.pickup_v1 import PickupV1
from positionSchemes.teleop_schemes.place_L4_v6_o import PlaceL4V6O
from positionSchemes.teleop_schemes.place_L2_v1 import PlaceL2V1
from positionSchemes.teleop_schemes.place_L1_v1 import PlaceL1V1
from positionSchemes.teleop_schemes.place_L3_v2_o import PlaceL3V1
from positionSchemes.teleop_schemes.put_L3_v2_o import PutL3V2O
from positionSchemes.teleop_schemes.put_L4_v1_o import PutL4V1O
from utils.signalLogging import addLog
from utils.singleton import Singleton

class PoseDirectorOperator(metaclass=Singleton):

    def __init__(self):
        self.common = PoseDirectorCommon()

    def initialize(self):
        self.common.controllerStateOperator = ElevArmCmdState.UNINITIALIZED
        self.common.prevControllerStateOperator = self.common.controllerStateOperator
        self.common.currentPositionSchemeOperator = PoserNoChangeOperator(self.common)
        self.getElevatorCommand = lambda curCommand :  self.common.currentPositionSchemeOperator.getElevatorCommand(curCommand)
        self.getArmCommand = lambda curCommand : self.common.currentPositionSchemeOperator.getArmCommand(curCommand)
        self.schemeProg = 0
        self.lastAutonToggle = 0
        self.whatToDo = ElevArmCmdState.L4
        self.scheme = None
        self.dashboardState = 1 # State 1, put the autonomous menu back up on the webserver dashboard
        addLog("RPO/schemeProg", lambda: self.schemeProg, "") # don't delete this.
        addLog("RPO/dashboardState", lambda: self.dashboardState, "") # don't delete this.
        # addLog("RP/controllerState", lambda: self.common.controllerStateOperator, "int")

    def setDashboardState(self, dashboardState: int):
        self.dashboardState = dashboardState

    def update(self, isAuton=False):

        if isAuton and self.lastAutonToggle != isAuton:
            scheme = self.common.auto.currentScheme
            if scheme is not None:
                if hasattr(scheme, "poser"):
                    if scheme.poser != 0:
                        self.scheme = scheme.poser
                        self.common.currentPositionSchemeOperator.deactivate()
                        self.common.currentPositionSchemeOperator = self.scheme
                        self.getElevatorCommand = lambda curCommand: self.scheme.getElevatorCommand(curCommand)
                        self.getArmCommand = lambda curCommand : self.scheme.getArmCommand(curCommand)
        else:
            if self._isControllerStateChanging() or self.lastAutonToggle != isAuton:
                self.common.currentPositionSchemeOperator.deactivate()
                self.common.currentPositionSchemeOperator = self.pickTheNewScheme()
                self.getElevatorCommand = lambda curCommand: self.common.currentPositionSchemeOperator.getElevatorCommand(curCommand)
                self.getArmCommand = lambda curCommand : self.common.currentPositionSchemeOperator.getArmCommand(curCommand)
            self.common.currentPositionSchemeOperator.update()
        self.lastAutonToggle = isAuton
        if hasattr(self.common.currentPositionSchemeOperator, "schemeProg"):
            self.schemeProg=round(self.common.currentPositionSchemeOperator.schemeProg*100)
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
                return PoserNoChangeOperator(self.common)
            case ElevArmCmdState.VEL_CONTROL:
                return PoserVelocityControlOperator(self.common)
            case ElevArmCmdState.PLUNGE:
                self.setDashboardState(5)
                return PlungeV1(self.common)
            case ElevArmCmdState.RECEIVE_CORAL:
                self.setDashboardState(4)
                return PickupV1(self.common)
            case ElevArmCmdState.L1:
                self.setDashboardState(6)
                self.whatToDo = ElevArmCmdState.L1
                return PlaceL1V1(self.common)
            case ElevArmCmdState.L2:
                self.setDashboardState(6)
                self.whatToDo = ElevArmCmdState.L2
                return PlaceL2V1(self.common)
            case ElevArmCmdState.L3:
                self.setDashboardState(6)
                self.whatToDo = ElevArmCmdState.L3
                return PlaceL3V1(self.common)
            case ElevArmCmdState.L4:
                self.setDashboardState(6)
                self.whatToDo = ElevArmCmdState.L4
                return PlaceL4V6O(self.common)
            case ElevArmCmdState.PUT:
                self.setDashboardState(6)
                if self.whatToDo==ElevArmCmdState.L3:
                    return PutL3V2O(self.common)
                else:
                    return PutL4V1O(self.common)
            case _:
                return PoserNoChangeOperator(self.common)




