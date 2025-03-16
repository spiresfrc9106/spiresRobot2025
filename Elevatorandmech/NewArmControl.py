# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Arm

# Notes to self (Benjamin) as of 3/5/2025:
#1) get motor to calibrate (set zero) at desired position
#2) get motor to not return to zero when joystick is relaxed
#3) get motor to only move + or - 90 degrees from its zero

from enum import IntEnum
import math
from time import sleep

from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer
from wpilib import TimedRobot

from Elevatorandmech.ArmCommand import ArmCommand
from Elevatorandmech.RobotPoser import PoseDirector
from utils.calibration import Calibration
from utils.robotIdentification import RobotIdentification, RobotTypes
from utils.signalLogging import  addLog, getNowLogger
from utils.singleton import Singleton
from utils.units import sign
from utils.units import wrapAngleDeg, wrapAngleRad
from wrappers.wrapperedRevThroughBoreEncoder import WrapperedRevThroughBoreEncoder
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class ArmDependentConstants:
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
                "ABS_SENSOR_MOUNT_OFFSET_DEG": -50.0 - 10,
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


armDepConstants = ArmDependentConstants().get()

ARM_M_CANID = armDepConstants['ARM_M_CANID']
ARM_M_INVERTED = armDepConstants['ARM_M_INVERTED']
ARM_M_CURRENT_LIMIT_A = armDepConstants['ARM_M_CURRENT_LIMIT_A']
ARM_GEARBOX_GEAR_RATIO = armDepConstants['ARM_GEARBOX_GEAR_RATIO']
ABS_SENSOR_MOUNT_OFFSET_DEG = armDepConstants['ABS_SENSOR_MOUNT_OFFSET_DEG']
ABS_SENSOR_INVERTED = armDepConstants['ABS_SENSOR_INVERTED']


#TODO Perhaps use the absolute encoder offset

MAX_ARM_POS_DEG = armDepConstants['MAX_ARM_POS_DEG']
MIN_ARM_POS_DEG = armDepConstants['MIN_ARM_POS_DEG']
MAX_ARM_VEL_DEGPS = armDepConstants['MAX_ARM_VEL_DEGPS']
MAX_ARM_ACCEL_DEGPS2 = armDepConstants['MAX_ARM_ACCEL_DEGPS2']

class ArmStates(IntEnum):
    UNINITIALIZED = 0
    OPERATING = 2

TIME_STEP_S = 0.02

class ArmControl(metaclass=Singleton):
    def __init__(self):
        # there will not be preset angles for heights,
        # it will just be going to the angle given by Noah's code

        self.name = "arm"

        self.state = ArmStates.UNINITIALIZED

        # please do not delete this xyzzy ask Yavin about this
        self.atAboutDown = False

        self.poseDirector = PoseDirector()

        self.manAdjMaxVoltage = Calibration(name=f"{self.name} Manual Adj Max Voltage", default=1.0, units="V")

        # Arm Motors
        self.motor = WrapperedSparkMax(ARM_M_CANID, f"{self.name}/motor", brakeMode=True,
                                       currentLimitA=int(ARM_M_CURRENT_LIMIT_A))
        motorIsInverted = ARM_M_INVERTED
        self.motor.setInverted(motorIsInverted)

        # Rev Relative Encoder
        self.encoder = WrapperedRevThroughBoreEncoder(port=9, name=f"{self.name}",
                                                      mountOffsetRad=math.radians(ABS_SENSOR_MOUNT_OFFSET_DEG),
                                                      dirInverted=ABS_SENSOR_INVERTED)

        # FF and proportional gain constants
        self.kV = Calibration(name="Arm kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Arm kS", default=0.1, units="V")
        self.kG = Calibration(name="Arm kG", default=0.25, units="V")
        self.kP = Calibration(name="Arm kP", default=0.4, units="V/rad error")  # Per 0.001 seconds
        self.maxPosDeg = Calibration(name="Arm Max Vel", default=MAX_ARM_POS_DEG, units="deg")
        self.minPosDeg = Calibration(name="Arm Max Vel", default=MIN_ARM_POS_DEG, units="deg")
        self.maxVelocityDegps = Calibration(name="Arm Max Vel", default=MAX_ARM_VEL_DEGPS, units="degps")
        self.maxAccelerationDegps2 = Calibration(name="Arm Max Accel", default=MAX_ARM_ACCEL_DEGPS2, units="degps2")

        self._initialized = False


    def _setActCurDesTrapPStates(self, posDeg, velDegps):
        self.actTrapPState = TrapezoidProfile.State(posDeg, velDegps)
        self.desTrapPState = TrapezoidProfile.State(posDeg, velDegps)
        self.curTrapPState = TrapezoidProfile.State(posDeg, velDegps)

    def _setActCurDesTrapPStatesToWhereTheArmIsNow(self):
        curArmPosDeg = self._getRelAngleWithOffsetDeg()
        self._setActCurDesTrapPStates(curArmPosDeg, 0.0)


    def initialize(self):

        if not self._initialized:
            # Set P gain on motor
            self.motor.setPID(self.kP.get(), 0.0, 0.0)
            # when we re-initialize tell motor to stay where it is at.
            self.motor.setVoltage(0)

            # units for trapezoidal will be degrees per second (velocity) and degrees per second squared (acceleration)
            self.trapProfiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxVelocityDegps.get(), self.maxAccelerationDegps2.get()))
            self._setActCurDesTrapPStates(0,0)

            self.stopped = False

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

            self.relEncOffsetAngleDeg = 0.0
            self.motorPosCmdRad = 0.0

            addLog(f"{self.name}/motor_pos_cmd_deg", lambda: math.degrees(self.motorPosCmdRad), "deg")
            addLog(f"{self.name}/abs_encoder_act_pos_deg", lambda: self.getAbsAngleDeg(), "deg")
            addLog(f"{self.name}/rel_encoder_offset_deg", lambda: self.relEncOffsetAngleDeg, "deg")
            addLog(f"{self.name}/state", lambda: self.state, "int")
            self.stateNowLogger = getNowLogger(f"{self.name}/stateNow", int)
            self.stateNowLogger.logNow(self.state)
            addLog(f"{self.name}/stopped", lambda: self.stopped, "bool")
            addLog(f"{self.name}/act_pos_deg", lambda: self.actTrapPState.position, "deg")
            addLog(f"{self.name}/act_vel_degps", lambda: self.actTrapPState.velocity, "degps")
            self.actAccLogger = getNowLogger(f"{self.name}/act acceleration", "degps2")
            addLog(f"{self.name}/curProfile_pos_deg", lambda: self.curTrapPState.position, "deg")
            addLog(f"{self.name}/curProfile_vel_degps", lambda: self.curTrapPState.velocity, "degps")
            self.curTrapPAccLogger = getNowLogger(f"{self.name}/curProfile_acc_degps2", "degps2")
            addLog(f"{self.name}/des_pos_deg", lambda: self.desTrapPState.position, "deg")
            addLog(f"{self.name}/des_vel_degps", lambda: self.desTrapPState.velocity, "degps")

            addLog("RParm/pos", lambda: self.curTrapPState.position, "deg")

            self.poserCmdPosLogger = getNowLogger(f"{self.name}/cmd_pos_deg", "deg")
            self.poserCmdVelLogger = getNowLogger(f"{self.name}/cmd_vel_degps", "degps")

            self._changeState(ArmStates.UNINITIALIZED)

            self.previousUpdateTimeS = None
            self.previousVelDegps = None

            self.count = 0

            self._initialized = True

            print(f"Init {self.name} complete")

    def disable(self):
        self.motor.setPosCmd(self.motor.getMotorPositionRad())
        self.motor.setVoltage(0)

    def _noOffsetMotorRadToAngleDeg(self, MotorRad: float) -> float:
        return  math.degrees(MotorRad * (1.0/ARM_GEARBOX_GEAR_RATIO))

    def _motorRadToAngleWithOffsetDeg(self, MotorRad: float) -> float:
        return  self._noOffsetMotorRadToAngleDeg(MotorRad) + self.relEncOffsetAngleDeg

    def _angleDegToMotorRad(self, armAngleDeg: float) -> float:
        return math.radians((armAngleDeg - self.relEncOffsetAngleDeg) * ARM_GEARBOX_GEAR_RATIO )
    
    def _angleVelDegpsToMotorVelRadps(self, armAngleDeg: float) -> float:
        return armAngleDeg * ARM_GEARBOX_GEAR_RATIO

    def _getRelAngleWithNoOffsetDeg(self) -> float:
        return self._noOffsetMotorRadToAngleDeg(self.motor.getMotorPositionRad())

    def _getRelAngleWithOffsetDeg(self) -> float:
        return self._motorRadToAngleWithOffsetDeg(self.motor.getMotorPositionRad())

    def _getVelocityDegps(self) -> float:
        return self._noOffsetMotorRadToAngleDeg(self.motor.getMotorVelocityRadPerSec())

    #return the angle of the arm as measured by the absolute sensor in angles
    def getAbsAngleDeg(self) -> float:
        angleAbsSenDeg = math.degrees(self.encoder.getAngleRad())
        #return angleAbsSenDeg - ABS_SENSOR_MOUNT_OFFSET_DEG
        return angleAbsSenDeg

    def getRelToAbsoluteSensorOffsetDeg(self) -> float:
        relAngleWithNoOffsetDeg = self._getRelAngleWithNoOffsetDeg()
        absAngleDeg = self.getAbsAngleDeg()

        relEncOffsetAngleDeg = absAngleDeg - relAngleWithNoOffsetDeg

        #if self.count % 1000 == 0:
        #    print(f"count={self.count} {relEncOffsetAngleDeg:+10.1f}=absAngleDeg({absAngleDeg:+10.1f} - relAngleWithNoOffsetDeg({relAngleWithNoOffsetDeg:+10.1f})\n... motorPositionRad={self.motor.getMotorPositionRad():+10.1f}->{self._noOffsetMotorRadToAngleDeg(self.motor.getMotorPositionRad()):+10.1f}")

        return relEncOffsetAngleDeg

    def update(self) -> None:
        self.encoder.update()

        match self.state:
            case ArmStates.UNINITIALIZED:
                self._updateUninitialized()
            case ArmStates.OPERATING:
                self._updateOperating()
            case _:
                pass

        self.count = self.count + 1

    def _updateUninitialized(self) -> None:
        self.encoder.update()

        self.trapProfiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxVelocityDegps.get(), self.maxAccelerationDegps2.get()))
        self.lastStoppedTimeS = 0
        self.relEncOffsetAngleDeg = self.getRelToAbsoluteSensorOffsetDeg()

        self._setActCurDesTrapPStatesToWhereTheArmIsNow()

        # when we re-initialize tell motor to stay where it is at.
        self.motor.setVoltage(0)
        vFF  = 0

        self.motor.setPosCmd(self.motor.getMotorPositionRad(), vFF)

        self._changeState(ArmStates.OPERATING)

    def _perhapsWeHaveANewRangeCheckedDesiredState(self, armCommand: ArmCommand)->TrapezoidProfile.State:

        newDesPosDeg = armCommand.angleDeg
        newDesVelocityDegps = armCommand.velocityDegps

        result = self.desTrapPState

        if (self.state == ArmStates.OPERATING) and (self.desTrapPState.position != newDesPosDeg or self.desTrapPState.velocity != newDesVelocityDegps):

            newDesVelocityDegps = min(newDesVelocityDegps, self.maxVelocityDegps.get())
            newDesVelocityDegps = max(newDesVelocityDegps, -self.maxVelocityDegps.get())

            if newDesPosDeg is not None:
                # limit the height goal so that it is less than max height
                # limit the height goal so that is more than 0
                newDesPosDeg = min(newDesPosDeg, self.maxPosDeg.get())
                newDesPosDeg = max(newDesPosDeg, self.minPosDeg.get())

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
                newDesPosDeg = self.maxPosDeg.get()
            elif newDesVelocityDegps < 0.0:
                newDesPosDeg = self.minPosDeg.get()
            else:
                newDesPosDeg = newDesPosDeg

            result = self.trapProfiler.State(newDesPosDeg, newDesVelocityDegps)

        return result


    def _setMotorPosAndFF(self) -> None:
        oldVelocityDegps = self.curTrapPState.velocity

        # This method is called both when initializing the object in the uninitialized state and when operating
        # In the initializing state we always have a target velocity at the end. In the operating state we might
        # have a target velocity that is not zero. If the target velocity is not zero we limit the velocity to be a
        # velocity that won't overrun the end limits based upon our current profiled position and the max acceleration.
        # Rather than solving with calculus, we just run an extra profiler and select the result with the smallest
        # magnitude of velocity.

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

        self.curTrapPAccLogger.logNow((self.curTrapPState.velocity - oldVelocityDegps) / TIME_STEP_S)

        motorPosCmdRad = self._angleDegToMotorRad(self.curTrapPState.position)
        motorVelCmdRadps = self._angleDegToMotorRad(self.curTrapPState.velocity)

        # set our feed forward to 0 at the start so we're not throwing extra voltage into the motor
        # then see if their feed forward calc makes sense
        vFF = self.kV.get() * motorVelCmdRadps  + self.kS.get() * sign(motorVelCmdRadps) \
            + self.kG.get()

        vFF = 0

        self.motor.setPosCmd(motorPosCmdRad, vFF)


    def _updateOperating(self) -> None:
        self.encoder.update()
        self.currentUpdateTimeS = Timer.getFPGATimestamp()
        self.actualVelDegps = self._getVelocityDegps()
        if self.previousUpdateTimeS is not None:
            currentPeriodS = self.currentUpdateTimeS - self.previousUpdateTimeS
            self.actAccLogger.logNow((self.actualVelDegps - self.previousVelDegps) / currentPeriodS)

        self.actTrapPState = TrapezoidProfile.State(self._getRelAngleWithOffsetDeg(), self.actualVelDegps)


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

        self.desTrapPState = self._perhapsWeHaveANewRangeCheckedDesiredState(armCommand)

        # Update motor closed-loop calibration
        if self.kP.isChanged():
            self.motor.setPID(self.kP.get(), 0.0, 0.0)

        # this doesn't happen right now
        if self.stopped:
            # Handle stopped by just holding mechanism in place with gravity offset, no closed loop.
            # TODO - do we need a more gentle stop here?
            manAdjVoltage = self.manAdjMaxVoltage.get() * self.manualAdjCmd

            self.motor.setVoltage(self.kG.get() + manAdjVoltage)
            self.curTrapPState = TrapezoidProfile.State(self.actTrapPState.position, 0)
        else:
            # this case does happen
            self._setMotorPosAndFF()

        self.previousVelDegps = self.actualVelDegps
        self.previousUpdateTimeS = self.currentUpdateTimeS

    # todo make an new API function:
    #def setPosVelocityGoal(self, posGoalDeg:float, velocityGoalDegps:float) -> None:
    #    self._perhapsWeHaveANewRangeCheckedDesiredState(newDesPosDeg=posGoalDeg, newDesVelocityDegs=velocityGoalDegps)

    def setManualAdjCmd(self, cmd:float) -> None:
        self.manualAdjCmd = cmd

    def _forceStartAtAngleZeroDeg(self) -> None:
        self.relEncOffsetRad = self.motor.getMotorPositionRad()

    def _changeState(self, newState: ArmStates) -> None:
        print(f"time = {Timer.getFPGATimestamp():.3f}s changing from arm state {self.state} to {newState}")
        self.state = newState
        self.stateNowLogger.logNow(self.state)

    # Yavin todo use these:
    def getCurProfilePosDeg(self) -> float:
        return self.curTrapPState.position

    def getCurProfileVelocityDegps(self) -> float:
        return self.curTrapPState.velocity

    def getPosition(self):
        return self.getCurProfilePosDeg()

    def getVelocity(self):
        return self.getCurProfileVelocityDegps()
