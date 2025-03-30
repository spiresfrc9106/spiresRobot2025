
from positionSchemes.teleop_schemes.defaultPosers import PoserNoChangeDriver, PoserVelocityControlDriver
from positionSchemes.RobotPoserCommon import PoseDirectorCommon
from humanInterface.operatorInterface import ElevArmCmdState
from positionSchemes.teleop_schemes.place_L4_v6_d import PlaceL4V6D
from positionSchemes.teleop_schemes.place_L3_v1 import PlaceL3V1
from positionSchemes.teleop_schemes.place_L2_v1 import PlaceL2V1
from positionSchemes.teleop_schemes.place_L1_v1 import PlaceL1V1
from utils.signalLogging import addLog
from utils.singleton import Singleton

class PoseDirectorDriver(metaclass=Singleton):
    
    def __init__(self):
        self.common = PoseDirectorCommon()

    def initialize(self):
        self.common.controllerStateDriver = ElevArmCmdState.UNINITIALIZED
        self.common.prevControllerStateDriver = self.common.controllerStateDriver
        self.common.currentPositionSchemeDriver = PoserNoChangeDriver(self.common)
        self.getDriveTrainCommand = lambda curCommand : self.common.currentPositionSchemeDriver.getDriveTrainCommand(curCommand)
        self.schemeProg = 0
        self.lastAutonToggle = 0
        self.dashboardState = 1 # State 1, put the autonomous menu back up on the webserver dashboard
        addLog("RPD/schemeProg", lambda: self.schemeProg, "") # don't delete this.
        addLog("RPD/dashboardState", lambda: self.dashboardState, "") # don't delete this.
        # addLog("RP/controllerState", lambda: self.common.controllerStateDriver, "int")

    def setDashboardState(self, dashboardState: int):
        self.dashboardState = dashboardState

    def update(self, isAuton=False):

        if isAuton:
            pass
        else:
            if self._isControllerStateChanging() or self.lastAutonToggle != isAuton:
                self.common.currentPositionSchemeDriver.deactivate()
                self.common.currentPositionSchemeDriver = self.pickTheNewScheme()
                self.getDriveTrainCommand = lambda curCommand : self.common.currentPositionSchemeDriver.getDriveTrainCommand(curCommand)
            self.common.currentPositionSchemeDriver.update()
        self.lastAutonToggle = isAuton
        if hasattr(self.common.currentPositionSchemeDriver, "schemeProg"):
            self.schemeProg=round(self.common.currentPositionSchemeDriver.schemeProg*100)
        else:
            self.setDashboardState(3)
        if isAuton:
            self.setDashboardState(2)

    def _isControllerStateChanging(self)->bool:
        nextState = self.common.dInt.getElevArmCmdState()
        changed = False
        if nextState != self.common.controllerStateDriver:
            print(
                f"RobotPoserDriver: state changing from {self.common.controllerStateDriver.name}({self.common.controllerStateDriver}) to {nextState.name} ({nextState})")
            self.common.prevControllerStateDriver = self.common.controllerStateDriver
            self.common.controllerStateDriver = nextState
            changed = True
        return changed

    def pickTheNewScheme(self)->None:
        match self.common.controllerStateDriver:
            case ElevArmCmdState.UNINITIALIZED:
                return PoserNoChangeDriver(self.common)
            case ElevArmCmdState.VEL_CONTROL:
                return PoserVelocityControlDriver(self.common)
            case ElevArmCmdState.PLUNGE:
                #self.setDashboardState(5)
                return None
            case ElevArmCmdState.RECEIVE_CORAL:
                return None
            case ElevArmCmdState.L1:
                self.setDashboardState(6)
                return PlaceL1V1(self.common)
            case ElevArmCmdState.L2:
                self.setDashboardState(6)
                return PlaceL2V1(self.common)
            case ElevArmCmdState.L3:
                self.setDashboardState(6)
                return PlaceL3V1(self.common)
            case ElevArmCmdState.L4:
                self.setDashboardState(6)
                return PlaceL4V6D(self.common)
            case _:
                return PoserNoChangeDriver(self.common)




