from enum import IntEnum
import math

from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer

from Elevatorandmech.ArmCommand import ArmCommand
from Elevatorandmech.RobotPoser import PoseDirector
from utils.calibration import Calibration
from utils.robotIdentification import RobotIdentification, RobotTypes
from utils.signalLogging import addLog, getNowLogger
from utils.singleton import Singleton


from Elevatorandmech.ElevatorControl import ElevatorControl, elevDepConstants
from Elevatorandmech.NewArmControl import ArmControl, armDepConstants
from humanInterface.operatorInterface import OperatorInterface


class ArmElevatorInterlockConstants:
    def __init__(self):

        self.armDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_ARM": False,
                "ARM_GEARBOX_GEAR_RATIO": 50.0 / 1.0,
                "ARM_M_CANID": None,
                "ARM_M_INVERTED": False,
                "ARM_M_CURRENT_LIMIT_A": 5,
                "MAX_ARM_POS_DEG": 90,
                "MIN_ARM_POS_DEG": -90,
                "MAX_ARM_VEL_DEGPS": 20,
                "MAX_ARM_ACCEL_DEGPS2": 4,
                "ABS_SENSOR_MOUNT_OFFSET_DEG": 0.0,
                "ABS_SENSOR_INVERTED": True,
            },
            RobotTypes.Spires2025: {
                "HAS_ARM": True,
                "ARM_GEARBOX_GEAR_RATIO": 50.0 / 1.0,
                "ARM_M_CANID": 23,
                "ARM_M_INVERTED": False,
                "ARM_M_CURRENT_LIMIT_A": 40,
                "MAX_ARM_POS_DEG": 80,
                "MIN_ARM_POS_DEG": -92,
                "MAX_ARM_VEL_DEGPS": 90, # Was 180
                "MAX_ARM_ACCEL_DEGPS2": 180, # Was 720
                "ABS_SENSOR_MOUNT_OFFSET_DEG": 160.0,
                "ABS_SENSOR_INVERTED": False,
            },
            RobotTypes.Spires2025Sim: {
                "HAS_ARM": True,
                "ARM_GEARBOX_GEAR_RATIO": 50.0 / 1.0,
                "ARM_M_CANID": 23,
                "ARM_M_INVERTED": True,
                "ARM_M_CURRENT_LIMIT_A": 5,
                "MAX_ARM_POS_DEG": 90,
                "MIN_ARM_POS_DEG": -92,
                "MAX_ARM_VEL_DEGPS": 90,
                "MAX_ARM_ACCEL_DEGPS2": 180,
                "ABS_SENSOR_MOUNT_OFFSET_DEG": 0.0,
                "ABS_SENSOR_INVERTED": False,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_ARM": True,
                "ARM_GEARBOX_GEAR_RATIO": 5.0 / 1.0,
                "ARM_M_CANID": 18,
                "ARM_M_INVERTED": True,
                "ARM_M_CURRENT_LIMIT_A": 5,
                "MAX_ARM_POS_DEG": 90,
                "MIN_ARM_POS_DEG": -90,
                "MAX_ARM_VEL_DEGPS": 90,
                "MAX_ARM_ACCEL_DEGPS2": 90,
                "ABS_SENSOR_MOUNT_OFFSET_DEG": -90.0,
                "ABS_SENSOR_INVERTED": True,
            },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_ARM": True,
                "ARM_GEARBOX_GEAR_RATIO": 5.0 / 1.0,
                "ARM_M_CANID": 18,
                "ARM_M_INVERTED": True,
                "ARM_M_CURRENT_LIMIT_A": 5,
                "MAX_ARM_POS_DEG": 90,
                "MIN_ARM_POS_DEG": -90,
                "MAX_ARM_VEL_DEGPS": 90,
                "MAX_ARM_ACCEL_DEGPS2": 90,
                "ABS_SENSOR_MOUNT_OFFSET_DEG": -90.0,
                "ABS_SENSOR_INVERTED": True,
            },
        }

    def get(self):
        return self.armDepConstants[RobotIdentification().getRobotType()]


armElevatorDepConstants = ArmElevatorInterlockConstants().get()

ARM_M_CANID = armElevatorDepConstants['ARM_M_CANID']
ARM_M_INVERTED = armElevatorDepConstants['ARM_M_INVERTED']
ARM_M_CURRENT_LIMIT_A = armElevatorDepConstants['ARM_M_CURRENT_LIMIT_A']
ARM_GEARBOX_GEAR_RATIO = armElevatorDepConstants['ARM_GEARBOX_GEAR_RATIO']
ABS_SENSOR_MOUNT_OFFSET_DEG = armElevatorDepConstants['ABS_SENSOR_MOUNT_OFFSET_DEG']
ABS_SENSOR_INVERTED = armElevatorDepConstants['ABS_SENSOR_INVERTED']


#TODO Perhaps use the absolute encoder offset

MAX_ARM_POS_DEG = armElevatorDepConstants['MAX_ARM_POS_DEG']
MIN_ARM_POS_DEG = armElevatorDepConstants['MIN_ARM_POS_DEG']
MAX_ARM_VEL_DEGPS = armElevatorDepConstants['MAX_ARM_VEL_DEGPS']
MAX_ARM_ACCEL_DEGPS2 = armElevatorDepConstants['MAX_ARM_ACCEL_DEGPS2']

class ArmStates(IntEnum):
    UNINITIALIZED = 0
    OPERATING = 2

TIME_STEP_S = 0.02

class ArmElevatorInterlock(metaclass=Singleton):
    def __init__(self):
        # there will not be preset angles for heights,
        # it will just be going to the angle given by Noah's code

        self.name = "armElevatorInterlock"




    def initialize(self, arm:ArmControl, elevator:ElevatorControl, oInt:OperatorInterface):

        self.arm = arm
        self.elevator = elevator
        self.oInt = oInt

        self.minArmPosDeg = self.arm.maxPosDeg
        self.maxArmPosDeg = self.arm.minPosDeg
        self.elevatorCanInitGoingDown = False
        self.minElevatorPosIn = self.elevator.maxPosIn


    def update(self):
        if self.arm.isOperating():
            if self.arm.getArmPosDeg()>0.0:
                self.elevatorCanInitGoingDown = True
                self.minElevatorPosIn = self.elevator.minPosIn
            elif self.arm.getArmPosDeg() > -85.0:
                self.elevatorCanInitGoingDown = False
                self.minElevatorPosIn = 10.0 # limit for low angles, no plunge
            else:
                self.elevatorCanInitGoingDown = False
                self.minElevatorPosIn = 8.0 # limit for low angles, during a plunge
        else:
            self.elevatorCanInitGoingDown = False

        if self.elevator.isOperating():
            if self.elevator.getElevatorPosIn() > 10.0:
                self.maxArmPosDeg = self.arm.maxPosDeg
                self.minArmPosDeg = self.arm.minPosDeg
            else:
                if self.arm.getArmPosDeg() < -85.0:
                    self.maxArmPosDeg = -85.0
                    self.minArmPosDeg = self.minArmPosDeg
                else:
                    self.minArmPosDeg = 0.0
                    self.maxArmPosDeg = self.maxArmPosDeg
        else:
            self.minArmPosDeg = self.arm.maxPosDeg
            self.maxArmPosDeg = self.arm.minPosDeg


