import wpilib
from utils.singleton import Singleton


# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Arm

# Notes to self (Benjamin) as of 3/5/2025:
#1) get motor to calibrate (set zero) at desired position
#2) get motor to not return to zero when joystick is relaxed
#3) get motor to only move + or - 90 degrees from its zero

from enum import IntEnum
import math

from wpimath.trajectory import TrapezoidProfile
import wpilib

from wpilib import Timer

from Elevatorandmech.ArmCommand import ArmCommand
from Elevatorandmech.RobotPoserOperator import PoseDirector
from utils.calibration import Calibration
from utils.robotIdentification import RobotIdentification, RobotTypes
from utils.signalLogging import  addLog, getNowLogger
from utils.singleton import Singleton
from utils.units import sign
from wrappers.wrapperedRevThroughBoreEncoder import WrapperedRevThroughBoreEncoder
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.motorStallDetector import MotorPosStallDetector

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
