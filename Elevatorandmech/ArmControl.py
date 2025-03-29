from enum import IntEnum
import math

from wpimath.trajectory import TrapezoidProfile
import wpilib
from wpilib import Timer

from Elevatorandmech.ArmCommand import ArmCommand
from positionSchemes.RobotPoserOperator import PoseDirectorOperator
from utils.calibration import Calibration
from utils.robotIdentification import RobotIdentification, RobotTypes
from utils.signalLogging import  addLog, getNowLogger
from utils.singleton import Singleton
from utils.units import sign
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.motorStallDetector import MotorPosStallDetector

class ArmDependentConstants:
    def __init__(self):

        self.armDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_ARM": False,
                "ARM_GEARBOX_GEAR_RATIO": 50.0 / 1.0,
                "ARM_M_CANID": None,
                "ARM_M_INVERTED": False,
                "ARM_M_INITIALIZING_CURRENT_LIMIT_A": 5,
                "ARM_M_OPERATING_CURRENT_LIMIT_A": 5,
                "ARM_ANGLE_AT_CURRENT_LIMIT_GOING_UP": 100,
                "MAX_ARM_POS_DEG": 90,
                "MIN_ARM_POS_DEG": -90,
                "MAX_SEARCH_ARM_VEL_DEGPS": 4,
                "MAX_SEARCH_ARM_ACCEL_DEGPS2": 4,
                "MAX_ARM_VEL_DEGPS": 20,
                "MAX_ARM_ACCEL_DEGPS2": 4,
                "ABS_SENSOR_INVERTED": True,
            },
            RobotTypes.Spires2025: {
                "HAS_ARM": True,
                "ARM_GEARBOX_GEAR_RATIO": 50.0 / 1.0,
                "ARM_M_CANID": 23,
                "ARM_M_INVERTED": False,
                "ARM_M_INITIALIZING_CURRENT_LIMIT_A": 10, # xyzzy CAUTION we're not using this yet
                "ARM_M_OPERATING_CURRENT_LIMIT_A": 60,
                "ARM_ANGLE_AT_CURRENT_LIMIT_GOING_UP": 93.9,
                "MAX_ARM_POS_DEG": 90,
                "MIN_ARM_POS_DEG": -92,
                "MAX_SEARCH_ARM_VEL_DEGPS": 60,
                "MAX_SEARCH_ARM_ACCEL_DEGPS2": 60*4,
                "MAX_ARM_VEL_DEGPS": 180*2,  # Was 180*2, 180 before
                "MAX_ARM_ACCEL_DEGPS2": 720*2,  # Was 720*2, 720 before
                "ABS_SENSOR_INVERTED": False,
            },
            RobotTypes.Spires2025Sim: {
                "HAS_ARM": True,
                "ARM_GEARBOX_GEAR_RATIO": 50.0 / 1.0,
                "ARM_M_CANID": 23,
                "ARM_M_INVERTED": True,
                "ARM_M_INITIALIZING_CURRENT_LIMIT_A": 5,
                "ARM_M_OPERATING_CURRENT_LIMIT_A": 5,
                "ARM_ANGLE_AT_CURRENT_LIMIT_GOING_UP": 100,
                "MAX_ARM_POS_DEG": 88,
                "MIN_ARM_POS_DEG": -89,
                "MAX_SEARCH_ARM_VEL_DEGPS": 60,
                "MAX_SEARCH_ARM_ACCEL_DEGPS2": 60*4,
                "MAX_ARM_VEL_DEGPS": 90,
                "MAX_ARM_ACCEL_DEGPS2": 180,
                "ABS_SENSOR_INVERTED": False,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_ARM": True,
                "ARM_GEARBOX_GEAR_RATIO": 5.0 / 1.0,
                "ARM_M_CANID": 18,
                "ARM_M_INVERTED": False,
                "ARM_M_INITIALIZING_CURRENT_LIMIT_A": 5,
                "ARM_M_OPERATING_CURRENT_LIMIT_A": 5,
                "ARM_ANGLE_AT_CURRENT_LIMIT_GOING_UP": 170,
                "MAX_ARM_POS_DEG": 160,
                "MIN_ARM_POS_DEG": -160,
                "MAX_SEARCH_ARM_VEL_DEGPS": 4,
                "MAX_SEARCH_ARM_ACCEL_DEGPS2": 4,
                "MAX_ARM_VEL_DEGPS": 90,
                "MAX_ARM_ACCEL_DEGPS2": 90,
                "ABS_SENSOR_INVERTED": False,
            },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_ARM": True,
                "ARM_GEARBOX_GEAR_RATIO": 5.0 / 1.0,
                "ARM_M_CANID": 18,
                "ARM_M_INVERTED": False,
                "ARM_M_INITIALIZING_CURRENT_LIMIT_A": 5,
                "ARM_M_OPERATING_CURRENT_LIMIT_A": 5,
                "ARM_ANGLE_AT_CURRENT_LIMIT_GOING_UP": 170,
                "MAX_ARM_POS_DEG": 160,
                "MIN_ARM_POS_DEG": -160,
                "MAX_SEARCH_ARM_VEL_DEGPS": 4,
                "MAX_SEARCH_ARM_ACCEL_DEGPS2": 4,
                "MAX_ARM_VEL_DEGPS": 90,
                "MAX_ARM_ACCEL_DEGPS2": 0,
                "ABS_SENSOR_INVERTED": False,
            },
        }

    def get(self):
        return self.armDepConstants[RobotIdentification().getRobotType()]


armDepConstants = ArmDependentConstants().get()

ARM_M_CANID = armDepConstants['ARM_M_CANID']
ARM_M_INVERTED = armDepConstants['ARM_M_INVERTED']
ARM_M_INITIALIZING_CURRENT_LIMIT_A = armDepConstants['ARM_M_INITIALIZING_CURRENT_LIMIT_A'] # xyzzy CAUTION we're not using this yet
ARM_M_OPERATING_CURRENT_LIMIT_A = armDepConstants['ARM_M_OPERATING_CURRENT_LIMIT_A']
ARM_GEARBOX_GEAR_RATIO = armDepConstants['ARM_GEARBOX_GEAR_RATIO']
ABS_SENSOR_INVERTED = armDepConstants['ABS_SENSOR_INVERTED']

ARM_ANGLE_AT_CURRENT_LIMIT_GOING_UP = armDepConstants['ARM_ANGLE_AT_CURRENT_LIMIT_GOING_UP']
MAX_ARM_POS_DEG = armDepConstants['MAX_ARM_POS_DEG']
MIN_ARM_POS_DEG = armDepConstants['MIN_ARM_POS_DEG']
MAX_SEARCH_ARM_VEL_DEGPS = armDepConstants['MAX_SEARCH_ARM_VEL_DEGPS']
MAX_SEARCH_ARM_ACCEL_DEGPS2 = armDepConstants['MAX_SEARCH_ARM_ACCEL_DEGPS2']
MAX_ARM_VEL_DEGPS = armDepConstants['MAX_ARM_VEL_DEGPS']
MAX_ARM_ACCEL_DEGPS2 = armDepConstants['MAX_ARM_ACCEL_DEGPS2']

class ArmStates(IntEnum):
    UNINITIALIZED = 0
    INIT_GOING_UP = 1
    OPERATING = 2
    NO_CMD = -1

TIME_STEP_S = 0.02


class ArmControl(metaclass=Singleton):
    def __init__(self):
        # there will not be preset angles for heights,
        # it will just be going to the angle given by Noah's code

        self.name = "arm"

        self.state = ArmStates.UNINITIALIZED

        # please do not delete this xyzzy ask Yavin about this
        self.atAboutDown = False

        self.poseDirector = PoseDirectorOperator()


        # FF and proportional gain constants
        self.kV = Calibration(name="Arm kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Arm kS", default=0.1, units="V")
        self.kG = Calibration(name="Arm kG", default=0.25, units="V")
        self.kP = Calibration(name="Arm kP", default=0.1, units="V/rad error")  # Per 0.001 seconds
        self.calMaxPosDeg = Calibration(name="Arm Max Vel", default=MAX_ARM_POS_DEG, units="deg")
        self.calMinPosDeg = Calibration(name="Arm Max Vel", default=MIN_ARM_POS_DEG, units="deg")
        self.calMaxVelocityDegps = Calibration(name="Arm Max Vel", default=MAX_ARM_VEL_DEGPS, units="degps")
        self.calMaxAccelerationDegps2 = Calibration(name="Arm Max Accel", default=MAX_ARM_ACCEL_DEGPS2, units="degps2")

        self.calSearchMaxVelocityDegps = Calibration(name="Arm Search Max Vel", default=MAX_SEARCH_ARM_VEL_DEGPS, units="degps")
        self.calSearchMaxAccelerationDegps2 = Calibration(name="Arm Search Max Accel", default=MAX_SEARCH_ARM_ACCEL_DEGPS2, units="degps2")

        self.calArmInitializingCurrentLimitA = Calibration(name="Arm INITIALIZING_CURRENT_LIMIT_A", default=ARM_M_INITIALIZING_CURRENT_LIMIT_A, units="A")
        self.calArmOperatingCurrentLimitA = Calibration(name="Arm OPERATING_CURRENT_LIMIT_A", default=ARM_M_OPERATING_CURRENT_LIMIT_A, units="A")

        self.calArmAngleAtCurrentLimitGoingUpDeg = Calibration(name="Arm AngleAtCurrentLimitGoingUpDeg", default=ARM_ANGLE_AT_CURRENT_LIMIT_GOING_UP, units="deg")

        self.stateNowLogger = getNowLogger(f"{self.name}/stateNow", int)
        self.stateNowLogger.logNow(ArmStates.UNINITIALIZED)
        self.poserCmdPosLogger = getNowLogger(f"{self.name}/cmd_pos_deg", "deg")
        self.poserCmdPosLogger.logNow(360)
        self.poserCmdVelLogger = getNowLogger(f"{self.name}/cmd_vel_degps", "degps")
        self.poserCmdVelLogger.logNow(0)

        # See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
        self.desTrapPState = TrapezoidProfile.State(90,0) # Angle 90 deg, Velocity 0 deg per s

        # Arm Motors
        self.motor = WrapperedSparkMax(ARM_M_CANID, f"{self.name}/motor", brakeMode=True, currentLimitA=self.calArmInitializingCurrentLimitA.get())
        motorIsInverted = ARM_M_INVERTED
        self.motor.setInverted(motorIsInverted)

        self.stallDectector = MotorPosStallDetector(f"{self.name}/stall", self.motor, stallCurrentLimitA=self.motor.currentLimitA, stallTimeLimitS=0.2)

        self._initialized = False


    def _setActCurDesTrapPStates(self, posDeg, velDegps):
        self.actTrapPState = TrapezoidProfile.State(posDeg, velDegps)
        self.desTrapPState = TrapezoidProfile.State(posDeg, velDegps)
        self.curTrapPState = TrapezoidProfile.State(posDeg, velDegps)

    def _setActCurDesTrapPStatesToWhereTheArmIsNow(self):
        curArmPosDeg = self._getRelAngleWithOffsetDeg()
        self._setActCurDesTrapPStates(curArmPosDeg, 0.0)

    def forceReInit(self):
        self._initialized = False

    def initialize(self):
        changed = self.calMaxPosDeg.isChanged() or \
                  self.calMinPosDeg.isChanged() or \
                  self.kP.isChanged() or \
                  self.calMaxVelocityDegps.isChanged() or \
                  self.calMaxAccelerationDegps2.isChanged() or \
                  self.calSearchMaxVelocityDegps.isChanged() or \
                  self.calSearchMaxAccelerationDegps2.isChanged() or \
                  self.calArmInitializingCurrentLimitA.isChanged() or \
                  self.calArmOperatingCurrentLimitA.isChanged() or \
                  self.calArmAngleAtCurrentLimitGoingUpDeg.isChanged()

        if changed or not self._initialized:
            # Set P gain on motor
            self.motor.setPID(self.kP.get(), 0.0, 0.0)
            # when we re-initialize tell motor to stay where it is at.
            self.motor.setVoltage(0)

            # units for trapezoidal will be degrees per second (velocity) and degrees per second squared (acceleration)

            self.minPosDeg = self.calMinPosDeg.get()
            self.maxPosDeg = self.calMaxPosDeg.get()
            self.trapProfiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.calSearchMaxVelocityDegps.get(), self.calSearchMaxAccelerationDegps2.get()))
            self._setActCurDesTrapPStates(0,0)

            # Try to set a small current limit and decide when we're on the bottom using this, and turn
            # off the motor when it doesn't need to spin anymore.  then up the current limit as needed


            # Relative Encoder Offsets
            # Relative encoders always start at 0 at power-on
            # However, we may or may not have the mechanism at the "zero" position when we powered on
            # These variables store an offset which is calculated from the absolute sensors
            # to make sure the relative sensors inside the encoders accurately reflect
            # the actual position of the mechanism
            self.relEncOffsetRad = 0.0

            # Create a motion profile with the given maximum velocity and maximum
            # acceleration constraints for the next setpoint.

            self.previousUpdateTimeS = None

            self.timeWhenChangeS = 0

            self._largestAngleDeg = -180.0

            self._loadMaxVelAndAccFromCal()
            #addLog(f"{self.name}/_largestAngleDeg", lambda: self._largestAngleDeg, "deg")
            addLog(f"{self.name}/state", lambda: self.state.value, "int")
            addLog(f"{self.name}/act_pos_deg", lambda: self.actTrapPState.position, "deg")
            #addLog(f"{self.name}/act_vel_degps", lambda: self.actTrapPState.velocity, "degps")
            #self.actAccLogger = getNowLogger(f"{self.name}/act acceleration", "degps2")
            addLog(f"{self.name}/curProfile_pos_deg", lambda: self.curTrapPState.position, "deg")
            addLog(f"{self.name}/curProfile_vel_degps", lambda: self.curTrapPState.velocity, "degps")
            #self.curTrapPAccLogger = getNowLogger(f"{self.name}/curProfile_acc_degps2", "degps2")
            #addLog(f"{self.name}/des_pos_deg", lambda: self.desTrapPState.position, "deg")
            #addLog(f"{self.name}/des_vel_degps", lambda: self.desTrapPState.velocity, "degps")

            addLog("RParm/pos", lambda: self.actTrapPState.position, "deg")

            self._changeState(ArmStates.UNINITIALIZED)

            self.previousUpdateTimeS = None
            self.previousVelDegps = None

            self.count = 0

            self._initialized = True

            print(f"Init {self.name} complete")

    def _loadMaxVelAndAccFromCal(self):
        self.maxVelocityDegps = self.calMaxVelocityDegps.get()
        self.maxAccelerationDegps2 = self.calMaxAccelerationDegps2.get()

    def _loadNewTrapProfiler(self):
        self.trapProfiler = TrapezoidProfile(
            TrapezoidProfile.Constraints(self.maxVelocityDegps,
                                         self.maxAccelerationDegps2))

    def hasMaxVelOrAccChanged(self):
        result = False
        if self.state == ArmStates.OPERATING:
            if self.calMaxVelocityDegps.isChanged() or self.calMaxAccelerationDegps2.isChanged():
                result = True
                self._loadMaxVelAndAccFromCal()
                self._loadNewTrapProfiler()
        return result
    def disable(self):
        self.motor.setPosCmd(self.motor.getMotorPositionRad())
        self.motor.setVoltage(0)

    def _noOffsetMotorRadToAngleDeg(self, MotorRad: float) -> float:
        return  math.degrees(MotorRad * (1.0/ARM_GEARBOX_GEAR_RATIO))

    def _motorRadToAngleWithOffsetDeg(self, MotorRad: float) -> float:
        return  self._noOffsetMotorRadToAngleDeg(MotorRad+self.relEncOffsetRad)

    def _angleDegToMotorRadNoOffset(self, armAngleDeg: float) -> float:
        return math.radians(armAngleDeg * ARM_GEARBOX_GEAR_RATIO )

    def _angleDegToMotorRad(self, armAngleDeg: float) -> float:
        return self._angleDegToMotorRadNoOffset(armAngleDeg) - self.relEncOffsetRad
    
    def _angleVelDegpsToMotorVelRadps(self, armAngleDeg: float) -> float:
        return armAngleDeg * ARM_GEARBOX_GEAR_RATIO

    def _getRelAngleWithNoOffsetDeg(self) -> float:
        return self._noOffsetMotorRadToAngleDeg(self.motor.getMotorPositionRad())

    def _getRelAngleWithOffsetDeg(self) -> float:
        return self._motorRadToAngleWithOffsetDeg(self.motor.getMotorPositionRad())

    def _getVelocityDegps(self) -> float:
        return self._noOffsetMotorRadToAngleDeg(self.motor.getMotorVelocityRadPerSec())

    def update(self) -> None:
        self.motor.getMotorPositionRad()
        self.stallDectector.update()

        match self.state:
            case ArmStates.UNINITIALIZED:
                self._updateUninitialized()
            case ArmStates.INIT_GOING_UP:
                self._updateInitGoingUp()
            case ArmStates.OPERATING:
                self._updateOperating()
            case ArmStates.NO_CMD:
                pass
            case _:
                pass

        self.count = self.count + 1

    def _updateUninitialized(self) -> None:
        self.startTime = Timer.getFPGATimestamp()
        self.timeWhenChangeS = Timer.getFPGATimestamp()
        self._changeState(ArmStates.INIT_GOING_UP)
        self._forceStartAtAngleDeg(0.0)

        if wpilib.RobotBase.isSimulation():
            self.desTrapPState = TrapezoidProfile.State(self._getRelAngleWithOffsetDeg(),0)
        else:
            self.desTrapPState = TrapezoidProfile.State(self._getRelAngleWithOffsetDeg()+720,0)
        self.curTrapPState = TrapezoidProfile.State(self._getRelAngleWithOffsetDeg(), 0)
        self._largestAngleDeg = self._getRelAngleWithOffsetDeg()
        self.motor.setSmartCurrentLimit(self.calArmInitializingCurrentLimitA.get())
        self._setMotorPosAndFF()

    def _updateInitGoingUp(self) -> None:
        positionDeg = self._getRelAngleWithOffsetDeg()

        if positionDeg > self._largestAngleDeg:
            self._largestAngleDeg = positionDeg
            self.timeWhenChangeS = Timer.getFPGATimestamp()
            self._setMotorPosAndFF()
            # change the time we last moved in seconds
        else:
            # because we didn't go any lower, maybe we have been at the lowest height for a second
            nowS = Timer.getFPGATimestamp()
            if nowS - 1 >= self.timeWhenChangeS:
                self._forceStartAtAngleDeg(self.calArmAngleAtCurrentLimitGoingUpDeg.get())
                self._loadNewTrapProfiler()
                self.desTrapPState = TrapezoidProfile.State(self._getRelAngleWithOffsetDeg(), 0)
                self.curTrapPState = TrapezoidProfile.State(self._getRelAngleWithOffsetDeg(), 0)
                self.motor.setSmartCurrentLimit(self.calArmOperatingCurrentLimitA.get())
                self.stallDectector.stallCurrentLimitA = self.motor.currentLimitA
                self._changeState(ArmStates.OPERATING)
            else:
                self._setMotorPosAndFF()

    def _setMotorPosAndFF(self, velocityCmd:bool=False, enablePosMove=True) -> None:
        oldVelocityDegps = self.curTrapPState.velocity

        # This method is called both when initializing the object in the uninitialized state and when operating
        # In the initializing state we always have a target velocity at the end. In the operating state we might
        # have a target velocity that is not zero. If the target velocity is not zero we limit the velocity to be a
        # velocity that won't overrun the end limits based upon our current profiled position and the max acceleration.
        # Rather than solving with calculus, we just run an extra profiler and select the result with the smallest
        # magnitude of velocity.

        if velocityCmd:
            desiredVelocityIsMaxVelocityProfiler = TrapezoidProfile(
                TrapezoidProfile.Constraints(abs(self.desTrapPState.velocity), self.maxAccelerationDegps2))
            possibleNextTrapPState = desiredVelocityIsMaxVelocityProfiler.calculate(TIME_STEP_S, self.curTrapPState, self.desTrapPState)
        else:
            possibleNextTrapPState = self.trapProfiler.calculate(TIME_STEP_S, self.curTrapPState, self.desTrapPState)

        if self.desTrapPState.velocity == 0:
            self.curTrapPState = possibleNextTrapPState
        else:
            perhapsSmallerNextTrapPState = self.trapProfiler.calculate(TIME_STEP_S, self.curTrapPState,
                                                                       TrapezoidProfile.State(
                                                                           position=self.desTrapPState.position,
                                                                           velocity=0))

            if abs(perhapsSmallerNextTrapPState.velocity) < abs(possibleNextTrapPState.velocity):
                self.curTrapPState = perhapsSmallerNextTrapPState
            else:
                self.curTrapPState = possibleNextTrapPState

        #self.curTrapPAccLogger.logNow((self.curTrapPState.velocity - oldVelocityDegps) / TIME_STEP_S)

        motorPosCmdRad = self._angleDegToMotorRad(self.curTrapPState.position)
        motorVelCmdRadps = self._angleDegToMotorRad(self.curTrapPState.velocity)

        # set our feed forward to 0 at the start so we're not throwing extra voltage into the motor
        # then see if their feed forward calc makes sense
        vFF = self.kV.get() * motorVelCmdRadps  + self.kS.get() * sign(motorVelCmdRadps) \
            + self.kG.get()

        vFF = 0

        if enablePosMove:
            self.motor.setPosCmd(motorPosCmdRad, vFF)
        else:
            self.motor.setVoltage(0)

    def _perhapsWeHaveANewRangeCheckedDesiredState(self, armCommand: ArmCommand)->TrapezoidProfile.State:

        newDesPosDeg = armCommand.angleDeg
        newDesVelocityDegps = armCommand.velocityDegps

        #result = self.desTrapPState
        result = (False, self.desTrapPState)

        if (self.state == ArmStates.OPERATING) and (self.desTrapPState.position != newDesPosDeg or self.desTrapPState.velocity != newDesVelocityDegps):

            isVelocityCmd = False
            newDesVelocityDegps = min(newDesVelocityDegps, self.calMaxVelocityDegps.get())
            newDesVelocityDegps = max(newDesVelocityDegps, -self.calMaxVelocityDegps.get())

            if newDesPosDeg is not None:
                # limit the height goal so that it is less than max height
                # limit the height goal so that is more than 0
                newDesPosDeg = min(newDesPosDeg, self.maxPosDeg)
                newDesPosDeg = max(newDesPosDeg, self.minPosDeg)

                # do these checks relative to the curTrapPState
                # if height goal is to go up, make sure that velocity goal is 0 or +
                if newDesPosDeg > self.curTrapPState.position:
                    newDesVelocityDegps = max(0, newDesVelocityDegps)

                # if height goals is to go down, make sure that the velocity goal is 0 or -
                elif newDesPosDeg < self.curTrapPState.position:
                    newDesVelocityDegps = min(0, newDesVelocityDegps)
                # if height goals is to stay at current height goal
                else:
                    newDesVelocityDegps = 0
            elif newDesVelocityDegps == 0.0:
                newDesPosDeg = self.curTrapPState.position
            elif newDesVelocityDegps > 0.0:
                newDesPosDeg = self.maxPosDeg
                isVelocityCmd = True
            elif newDesVelocityDegps < 0.0:
                newDesPosDeg = self.minPosDeg
                isVelocityCmd = True
            else:
                newDesPosDeg = newDesPosDeg

            result = (isVelocityCmd, TrapezoidProfile.State(newDesPosDeg, newDesVelocityDegps))

        return result

    def _updateOperating(self) -> None:
        self.currentUpdateTimeS = Timer.getFPGATimestamp()
        self.actualVelDegps = self._getVelocityDegps()
        if self.previousUpdateTimeS is not None:
            currentPeriodS = self.currentUpdateTimeS - self.previousUpdateTimeS
            #self.actAccLogger.logNow((self.actualVelDegps - self.previousVelDegps) / currentPeriodS)

        self.actTrapPState = TrapezoidProfile.State(self._getRelAngleWithOffsetDeg(), self.actualVelDegps)

        self.hasMaxVelOrAccChanged()
        # default to not moving
        posGoalDeg = self.curTrapPState.position
        velocityGoalDegps = 0.0

        armCommand = ArmCommand(posGoalDeg, velocityGoalDegps)

        armCommand = self.poseDirector.getArmCommand(armCommand)

        if armCommand.angleDeg is None:
            self.poserCmdPosLogger.logNow(360)
        else:
            self.poserCmdPosLogger.logNow(armCommand.angleDeg)

        self.poserCmdVelLogger.logNow(armCommand.velocityDegps)

        velocityCmd, self.desTrapPState = self._perhapsWeHaveANewRangeCheckedDesiredState(armCommand)

        # Update motor closed-loop calibration
        if self.kP.isChanged():
            self.motor.setPID(self.kP.get(), 0.0, 0.0)

        self._setMotorPosAndFF(velocityCmd=velocityCmd, enablePosMove=True)

        self.previousVelDegps = self.actualVelDegps
        self.previousUpdateTimeS = self.currentUpdateTimeS



    def _forceStartAtAngleDeg(self, angleDeg:float) -> None:
        self.relEncOffsetRad = self._angleDegToMotorRadNoOffset(angleDeg) - self.motor.getMotorPositionRad()

    def _changeState(self, newState: ArmStates) -> None:
        print(f"time = {Timer.getFPGATimestamp():.3f}s changing from arm state {self.state} to {newState}")
        self.state = newState
        self.stateNowLogger.logNow(self.state)

    # Yavin todo use these:
    def getCurProfilePosDeg(self) -> float:
        return self.actTrapPState.position

    def getCurProfileVelocityDegps(self) -> float:
        return self.actTrapPState.velocity

    def getPosition(self):
        return self.getCurProfilePosDeg()

    def getVelocity(self):
        return self.getCurProfileVelocityDegps()


