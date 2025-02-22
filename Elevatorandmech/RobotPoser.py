# from humanInterface.driverInterface import DriverInterface
# from humanInterface.operatorInterface import OperatorInterface
# from Elevatorandmech.ArmControl import ArmControl
# from Elevatorandmech.ElevatorControl import ElevatorControl
# from drivetrain.drivetrainControl import DrivetrainControl

"""
NOAH TASK:

Make a class in a new file, 
two classes: SignalDirector and RobotPoser

Both classes have __init__ functions

for SignalDirector: 
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
from utils.singleton import Singleton

controllerStates = {
    "PositionControl" : 0,
    "VelocityControl" : 1,
    "ReceiveCoral" : 2,
    "Plunge" : 3,
    "L1" : 4,
    "L2" : 5,
    "L3" : 6,
    "L4" : 7,
}

class SignalDirector(metaclass=Singleton):
    def __init__(self):
        self.controllerState = controllerStates["PositionControl"]

    def update(self):
        pass

    def disableJoySticks(self):
        return 0

    def setControllerState(self, state):
        self.controllerState = controllerStates[state]

    def getControllerState(self):
        return self.controllerState


class RobotPoser:
    def __init__(self):
        pass

    def update(self):
        # x = receive from coral station pose
        # y = plunge
        # LeftTrigger = place at L1
        # RightTrigger = place at L2
        # A = place at L3
        # b = place at L4
        pass

