
from positionSchemes.defaultPosers import YavinsPoseClassNoChangeDriver, YavinsPoseClassVelocityControlDriver
from Elevatorandmech.RobotPoserCommon import PoseDirectorCommon
from humanInterface.operatorInterface import ElevArmCmdState
from positionSchemes.pickup_v1 import PickupV1
from positionSchemes.place_L4_v6_d import PlaceL4V6D
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

class PoseDirectorDriver(metaclass=Singleton):
    
    def __init__(self):
        self.common = PoseDirectorCommon()

    def initialize(self):

        self.common.controllerStateDriver = ElevArmCmdState.UNINITIALIZED
        self.common.prevControllerStateDriver = self.common.controllerStateDriver
        self.common.currentPositionSchemeDriver = YavinsPoseClassNoChangeDriver(self.common.driveTrain, self.common.dInt)
        self.getDriveTrainCommand = lambda curCommand : self.common.currentPositionSchemeDriver.getDriveTrainCommand(curCommand)
        self.schemeProg = 0
        self.dashboardState = 1 # State 1, put the autonomous menu back up on the webserver dashboard
        addLog("RPD/schemeProg", lambda: self.schemeProg, "") # don't delete this.
        addLog("RPD/dashboardState", lambda: self.dashboardState, "") # don't delete this.
        # addLog("RP/controllerState", lambda: self.common.controllerStateDriver, "int")

    def setDashboardState(self, dashboardState: int):
        self.dashboardState = dashboardState

    def update(self, isAuton=False):

        if (not isAuton) and self._isControllerStateChanging():
            self.common.currentPositionSchemeDriver.deactivate()
            self.common.currentPositionSchemeDriver = self.pickTheNewScheme()
            self.getDriveTrainCommand = lambda curCommand : self.common.currentPositionSchemeDriver.getDriveTrainCommand(curCommand)

        self.common.currentPositionSchemeDriver.update()
        if hasattr(self.common.currentPositionSchemeDriver, "schemeProg"):
            self.schemeProg=round(self.common.currentPositionSchemeDriver.schemeProg*100)
            # xyzzy, todo ask Yavin, shouldn't we set a dashboard state here?
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
                return YavinsPoseClassNoChangeDriver(self.common.driveTrain,  self.common.dInt)
            case ElevArmCmdState.VEL_CONTROL:
                return YavinsPoseClassVelocityControlDriver(self.common.driveTrain, self.common.dInt) 
            case ElevArmCmdState.PLUNGE:
                #self.setDashboardState(5)
                return None
            case ElevArmCmdState.RECEIVE_CORAL:
                self.setDashboardState(4)
                return PickupV1(self.common.driveTrain, self.common.dInt)
            case ElevArmCmdState.L1:
                self.setDashboardState(6)
                return PlaceL1V1(self.common.driveTrain, self.common.dInt)
            case ElevArmCmdState.L2:
                self.setDashboardState(6)
                return PlaceL2V1(self.common.driveTrain, self.common.dInt)
            case ElevArmCmdState.L3:
                self.setDashboardState(6)
                return PlaceL3V1(self.common.driveTrain, self.common.dInt)
            case ElevArmCmdState.L4:
                self.setDashboardState(6)
                return PlaceL4V6D(self.common)
            case _:
                return YavinsPoseClassNoChangeDriver(self.common.driveTrain, self.oInt)  




