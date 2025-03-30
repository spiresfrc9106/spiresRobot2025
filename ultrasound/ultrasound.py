
import wpilib

from utils.robotIdentification import RobotIdentification, RobotTypes
from utils.signalLogging import  addLog, getNowLogger
from utils.singleton import Singleton
from Elevatorandmech.ElevatorControl import ElevatorControl
from Elevatorandmech.ArmControl import ArmControl


class UltrasoundDependentConstants:
    def __init__(self):

        self.ultasoundDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_ARM_ULTRASOUND": False,
                "ARM_ULTRASOUND_ANALOGIO": 0,
                "HAS_LEFT_ULTRASOUND": False,
                "LEFT_ULTRASOUND_ANALOGIO": 1,
                "HAS_RIGHT_ULTRASOUND": False,
                "RIGHT_ULTRASOUND_ANALOGIO": 2,
            },
            RobotTypes.Spires2025: {
                "HAS_ARM_ULTRASOUND": True,
                "ARM_ULTRASOUND_ANALOGIO": 0,
                "HAS_LEFT_ULTRASOUND": True,
                "LEFT_ULTRASOUND_ANALOGIO": 1,
                "HAS_RIGHT_ULTRASOUND": True,
                "RIGHT_ULTRASOUND_ANALOGIO": 2,
            },
            RobotTypes.Spires2025Sim: {
                "HAS_ARM_ULTRASOUND": True,
                "ARM_ULTRASOUND_ANALOGIO": 0,
                "HAS_LEFT_ULTRASOUND": True,
                "LEFT_ULTRASOUND_ANALOGIO": 1,
                "HAS_RIGHT_ULTRASOUND": True,
                "RIGHT_ULTRASOUND_ANALOGIO": 2,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_ARM_ULTRASOUND": True,
                "ARM_ULTRASOUND_ANALOGIO": 0,
                "HAS_LEFT_ULTRASOUND": False,
                "LEFT_ULTRASOUND_ANALOGIO": 1,
                "HAS_RIGHT_ULTRASOUND": False,
                "RIGHT_ULTRASOUND_ANALOGIO": 2,
            },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_ARM_ULTRASOUND": False,
                "ARM_ULTRASOUND_ANALOGIO": 0,
                "HAS_LEFT_ULTRASOUND": False,
                "LEFT_ULTRASOUND_ANALOGIO": 1,
                "HAS_RIGHT_ULTRASOUND": False,
                "RIGHT_ULTRASOUND_ANALOGIO": 2,
            },
        }

    def get(self):
        return self.ultasoundDepConstants[RobotIdentification().getRobotType()]


ultrasoundDepConstants = UltrasoundDependentConstants().get()


class Ultrasound(metaclass=Singleton):
    def __init__(self):
        
        self.armDistanceIn  = 0.0
        self.leftFrontDistanceIn = 0.0
        self.rightFrontDistanceIn = 0.0
        
        if ultrasoundDepConstants['HAS_ARM_ULTRASOUND']:
            self.armAI = wpilib.AnalogInput(ultrasoundDepConstants['ARM_ULTRASOUND_ANALOGIO'])
            addLog("ultrasound/armDistanceIn", lambda : self.armDistanceIn, "in")
        else:
            self.armAI = None
        if ultrasoundDepConstants['HAS_LEFT_ULTRASOUND']:
            self.leftAI = wpilib.AnalogInput(ultrasoundDepConstants['LEFT_ULTRASOUND_ANALOGIO'])
            addLog("ultrasound/leftFrontDistanceIn", lambda : self.leftFrontDistanceIn, "in")
        else:
            self.leftAI = None
        if ultrasoundDepConstants['HAS_RIGHT_ULTRASOUND']:
            self.rightAI = wpilib.AnalogInput(ultrasoundDepConstants['RIGHT_ULTRASOUND_ANALOGIO'])
            addLog("ultrasound/rightFrontDistanceIn", lambda : self.rightFrontDistanceIn, "in")
        else:
            self.rightAI = None
            


    @classmethod
    def readDistance(cls,
                     analogInput:wpilib.AnalogInput):
        currentDistanceInches = None
        if analogInput is not None:
            rawVolts = analogInput.getValue()
    
            voltage_scale_factor = 5 / wpilib.RobotController.getVoltage5V()
            currentDistanceInches = rawVolts * voltage_scale_factor * 0.0492
        return currentDistanceInches
    
    def readArmDistance(self):
        self.armDistanceIn = self.readDistance(self.armAI)
        return self.armDistanceIn

    def readLeftFrontDistance(self):
        self.leftFrontDistanceIn = self.readDistance(self.leftAI)
        return self.leftFrontDistanceIn

    def readRightFrontDistance(self):
        self.rightFrontDistanceIn = self.readDistance(self.leftAI)
        return self.rightFrontDistanceIn
    
    def update(self):
        self.readArmDistance()
        self.readLeftFrontDistance()
        self.readRightFrontDistance()

class CoralDetection:

    def __init__(self):
        self.ultrasound = Ultrasound()
        self.elevCtrl = ElevatorControl()
        self.armCtrl = ArmControl()


    def hasCoral(self):
        if 16.0 < self.ultrasound.readArmDistance() < 20.0:
            #Has A coral ready for Placement
            return True
        return False

    def checkIfCoralPlaced(self) -> bool:
        if 20.0 < self.ultrasound.readArmDistance() < 24.0:
            #True when Coral has just placed
            return True
        return False

    def checkElevPlaceL4(self) -> bool:
        if 59.0 < self.elevCtrl.getCurProfilePosIn() < 61.0:
            #Elev at a level where it is able to place
            return True
        return False

    def checkArmPlaceL4(self) -> bool:
        if self.armCtrl.getCurProfilePosDeg() < 0.0:
            #Arm is going down and has just placed, assuming there was already coral
            return True
        return False

    def hasJustPlacedL4(self, code: str) -> bool:
        if self.checkIfCoralPlaced() and self.checkElevPlaceL4() and self.checkArmPlaceL4():
            #Coral has just been placed
            #Will only be true just after placing
            return True
        print(code)
        return False

    def update(self):
        print("YavinYavinYavinYavinYaivin")
        pass

