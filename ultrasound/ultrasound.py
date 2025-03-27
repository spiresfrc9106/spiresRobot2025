
import wpilib

from utils.robotIdentification import RobotIdentification, RobotTypes
from utils.signalLogging import  addLog, getNowLogger
from utils.singleton import Singleton


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
