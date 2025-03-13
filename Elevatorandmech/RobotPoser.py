
"""
NOAH TASK:

Make a class in a new file, 
two classes:  and RobotPoser

Both classes have __init__ functions

for :
Init class will take driver interface, operator interface object, drivetrain object, elevator object, 
arm object, going to know what button is being held down, if a button is being held down then it will 
know to route the signal to only the buttons, Joysticks should not work when buttons held down, 2nd 
version of this might be that the joysticks control will be heavily diminished, is the signal router class a 
subclass of the poser might establish hierarchy between each, each of these objects have update class that will 
be called in teleop, classes will also have an update that will be called it autonomous, 

--

RobotPoser: Take an operator interface object, some buttons on the operator interface will cause the arm 
and elevator to go into different poses, poses are Receive coral, complete a plunge, final poses you're 
gonna get to are receive from coral, complete a plunge operation on coral, *assume driving in complete plunge pose*, 
place at L2, L3, L4, for some of the poses you're going to override the driver control of steering the robot, 
some you're going to override the operator control, while operator is holding the place on that level button, 
will keep on doing the procedure as long as the button is held down, person can abort the procedure once button lifted up,   
"""


from Elevatorandmech.replaceWithYavinsPosesClass import YavinsPoseClassNoChange, YavinsPoseClassPositionControl, YavinsPoseClassVelocityControl
from humanInterface.operatorInterface import OperatorInterface, ElevArmCmdState
from positionSchemes.plunge_v1 import PlungeV1
from positionSchemes.pickup_v1 import PickupV1
from positionSchemes.place_L4_v1 import PlaceL4V1
from utils.signalLogging import addLog
from utils.singleton import Singleton

class PoseDirector(metaclass=Singleton):

    def initialize(self, arm, driveTrain, elevator):
        self.arm = arm
        self.driveTrain = driveTrain
        self.elevator = elevator
        self.oInt = OperatorInterface()
        self.controllerState = ElevArmCmdState.UNINITIALIZED
        self.prevControllerState = self.controllerState
        self.currentPositionScheme = YavinsPoseClassNoChange(self.arm, self.driveTrain, self.elevator)
        self.getDriveTrainCommand = lambda curCommand : self.currentPositionScheme.getDriveTrainCommand(curCommand)
        self.getElevatorCommand = lambda curCommand :  self.currentPositionScheme.getElevatorCommand(curCommand)
        self.getArmCommand = lambda curCommand : self.currentPositionScheme.getArmCommand(curCommand)
        addLog("RP/controllerState", lambda: self.controllerState, "int")

    def update(self, isAuton=False):
        if self._isControllerStateChanging():
            self.currentPositionScheme = self.pickTheNewScheme()
            self.getDriveTrainCommand = lambda curCommand : self.currentPositionScheme.getDriveTrainCommand(curCommand)
            self.getElevatorCommand = lambda curCommand: self.currentPositionScheme.getElevatorCommand(curCommand)
            self.getArmCommand = lambda curCommand : self.currentPositionScheme.getArmCommand(curCommand)
        self.currentPositionScheme.update()

    def _isControllerStateChanging(self)->bool:
        nextState = self.oInt.getElevArmCmdState()
        if nextState==ElevArmCmdState.VEL_CONTROL or nextState == ElevArmCmdState.POS_CONTROL:
            self.defaultJoystickMovement = nextState
        changed = False
        if nextState != self.controllerState:
            print(
                f"RobotPoser: state changing from {self.controllerState.name}({self.controllerState}) to {nextState.name} ({nextState})")
            self.prevControllerState = self.controllerState
            self.controllerState = nextState
            changed = True
        return changed

    def pickTheNewScheme(self)->None:
        match self.controllerState:
            case ElevArmCmdState.UNINITIALIZED:
                return YavinsPoseClassNoChange(self.arm, self.driveTrain, self.elevator)
            case ElevArmCmdState.VEL_CONTROL:
                return YavinsPoseClassVelocityControl(self.arm, self.driveTrain, self.elevator) # todo fix me
            case ElevArmCmdState.POS_CONTROL:
                return YavinsPoseClassPositionControl(self.arm, self.driveTrain, self.elevator)
            case ElevArmCmdState.PLUNGE:
                return PlungeV1(self.arm, self.driveTrain, self.elevator) # todo fix me
            case ElevArmCmdState.RECEIVE_CORAL:
                return PickupV1(self.arm, self.driveTrain, self.elevator) # todo fix me
            case ElevArmCmdState.L1:
                return YavinsPoseClassNoChange(self.arm, self.driveTrain, self.elevator)  # todo fix me
            case ElevArmCmdState.L2:
                return YavinsPoseClassNoChange(self.arm, self.driveTrain, self.elevator)  # todo fix me
            case ElevArmCmdState.L3:
                return YavinsPoseClassNoChange(self.arm, self.driveTrain, self.elevator)  # todo fix me
            case ElevArmCmdState.L4:
                return PlaceL4V1(self.arm, self.driveTrain, self.elevator)  # todo fix me
            case _:
                return YavinsPoseClassNoChange(self.arm, self.driveTrain, self.elevator)  # todo fix me

