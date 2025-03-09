# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Arm

# Notes to self (Benjamin) as of 3/5/2025:
#1) get motor to calibrate (set zero) at desired position
#2) get motor to not return to zero when joystick is relaxed
#3) get motor to only move + or - 90 degrees from its zero

from enum import IntEnum
from time import sleep

from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer
from wpilib import TimedRobot
import math


from utils.calibration import Calibration
from utils.units import sign
from utils.signalLogging import  addLog, getNowLogger
from utils.singleton import Singleton
from utils.robotIdentification import RobotIdentification
from wrappers.wrapperedRevThroughBoreEncoder import WrapperedRevThroughBoreEncoder
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from utils.robotIdentification import RobotTypes
from utils.units import wrapAngleDeg, wrapAngleRad

class ArmDependentConstants:
    def __init__(self):
        self.armDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_ARM": False,
                "ARM_GEARBOX_GEAR_RATIO": 50.0 / 1.0,
                "ARM_M_CANID": None,
                "MAX_ARM_VEL_DEGPS": 20,
                "MAX_ARM_ACCEL_DEGPS2": 4,
                "ABS_SENSOR_MOUNT_OFFSET_DEG": 0.0,
            },
            RobotTypes.Spires2025: {
                "HAS_ARM": True,
                "ARM_GEARBOX_GEAR_RATIO": 50.0 / 1.0,
                "ARM_M_CANID": 99,  # xyzzy fix me
                "MAX_ARM_VEL_DEGPS": 20,
                "MAX_ARM_ACCEL_DEGPS2": 4,
                "ABS_SENSOR_MOUNT_OFFSET_DEG": 0.0,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_ARM": False,  # xyzzy talk to Benjamin about switching dev test setups
                "ARM_GEARBOX_GEAR_RATIO": 5.0 / 1.0,
                "ARM_M_CANID": 99,  # xyzzy fix me
                "MAX_ARM_VEL_DEGPS": 20,
                "MAX_ARM_ACCEL_DEGPS2": 4,
                "ABS_SENSOR_MOUNT_OFFSET_DEG": 0.0,
            },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_ARM": True,
                "ARM_GEARBOX_GEAR_RATIO": 5.0 / 1.0,
                "ARM_M_CANID": 18,
                "MAX_ARM_VEL_DEGPS": 90,
                "MAX_ARM_ACCEL_DEGPS2": 90,
                "ABS_SENSOR_MOUNT_OFFSET_DEG": -90.0,
            },
        }

    def get(self):
        return self.armDepConstants[RobotIdentification().getRobotType()]


armDepConstants = ArmDependentConstants().get()

ARM_GEARBOX_GEAR_RATIO = armDepConstants['ARM_GEARBOX_GEAR_RATIO']
ABS_SENSOR_MOUNT_OFFSET_DEG = armDepConstants['ABS_SENSOR_MOUNT_OFFSET_DEG']
#TODO it would be interesting to try using above offset but in rads
# in defining the Abs encoder as the encoder does have the option for mount offset
MAX_ARM_VEL_DEGPS = armDepConstants['MAX_ARM_VEL_DEGPS']
MAX_ARM_ACCEL_DEGPS2 = armDepConstants['MAX_ARM_ACCEL_DEGPS2']


class ArmStates(IntEnum):
    ARM_UNINITIALIZED = 0
    ARM_OPERATING = 2


class ArmControl(metaclass=Singleton):
    def __init__(self):
        # there will not be preset angles for heights,
        # it will just be going to the angle given by Noah's code

        self.name = "arm"

        self.manAdjMaxVoltage = Calibration(name="Arm Manual Adj Max Voltage", default=1.0, units="V")

        self.armGoalDeg = 0.0
        
        # See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
        self.desTrapPState = TrapezoidProfile.State(self.armGoalDeg,0)

        # Arm Motors
        self.motor = WrapperedSparkMax(armDepConstants['ARM_M_CANID'], "ArmMotor", brakeMode=False, currentLimitA=5)
        MotorIsInverted = True
        self.motor.setInverted(MotorIsInverted)

        # Rev Relative Encoder
        self.encoder = WrapperedRevThroughBoreEncoder(port=9, name="ArmRevAbsEncoder", mountOffsetRad=0, dirInverted=True)

        # FF and proportional gain constants
        self.kV = Calibration(name="Arm kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Arm kS", default=0.1, units="V")
        self.kG = Calibration(name="Arm kG", default=0.25, units="V")
        self.kP = Calibration(name="Arm kP", default=0.4, units="V/rad error") # Per 0.001 seconds

        # Set P gain on motor
        self.motor.setPID(self.kP.get(), 0.0, 0.0)

        # Profiler
        #change eg velocity inches per second to degrees per second
        self.maxVelocityDegps = Calibration(name="Arm Max Vel", default=MAX_ARM_VEL_DEGPS, units="degps")
        self.maxAccelerationDegps2 = Calibration(name="Arm Max Accel", default=MAX_ARM_ACCEL_DEGPS2, units="degps2")
        
        #no need for search, so it can go away
        #will read abs encoder instead

        # units for trapezoidal will be degrees per second (velocity) and degrees per second squared (acceleration)
        self.trapProfiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxVelocityDegps.get(), self.maxAccelerationDegps2.get()))
        self.actTrapPState = self.trapProfiler.State()
        self.curTrapPState = self.trapProfiler.State()
        #go to wpilib online documentation to learn more about the trapezoid (very cool)

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

        self.armGoalDeg = 0.0
        self.relEncOffsetAngleDeg = 0.0
        self.motorPosCmdRad = 0.0

        self.state = ArmStates.ARM_UNINITIALIZED

        addLog(f"{self.name}/motor_pos_cmd_deg", lambda: math.degrees(self.motorPosCmdRad), "deg")
        addLog(f"{self.name}/abs_encoder_act_pos_deg", lambda: self.getAbsAngleDeg(), "deg")
        addLog(f"{self.name}/rel_encoder_offset_deg", lambda: self.relEncOffsetAngleDeg, "deg")
        addLog(f"{self.name}/state", lambda: self.state, "int")
        addLog(f"{self.name}/goal_pos_deg", lambda: self.armGoalDeg, "deg")
        addLog(f"{self.name}/stopped", lambda: self.stopped, "bool")
        addLog(f"{self.name}/act_pos_degps", lambda: self.actTrapPState.position, "deg")
        addLog(f"{self.name}/act_vel_degps", lambda: self.actTrapPState.velocity, "degps")
        self.actAccLogger = getNowLogger(f"{self.name}/act Acceleration", "degps2")
        addLog(f"{self.name}/curProfile_pos_Deg", lambda: self.curTrapPState.position, "deg")
        addLog(f"{self.name}/curProfile_vel_Degps", lambda: self.curTrapPState.velocity, "degps")
        self.curTrapPAccLogger = getNowLogger(f"{self.name}/curProfile_acc_degps2", "degps2")
        addLog(f"{self.name}/des_pos_deg", lambda: self.desTrapPState.position, "deg")
        addLog(f"{self.name}/des_vel_degps", lambda: self.desTrapPState.velocity, "degps")

        self.profiledPos = 0.0
        self.curUnprofiledPosCmd = 0.0
        self.previousUpdateTimeS = None
        self.previousVelDegps = None

        self.count = 0

        print(f"Init {self.name} complete")

    def _noOffsetMotorRadToAngleDeg(self, MotorRad: float) -> float:
        return  math.degrees(MotorRad * (1.0/ARM_GEARBOX_GEAR_RATIO))

    def _motorRadToAngleWithOffsetDeg(self, MotorRad: float) -> float:
        return  self._noOffsetMotorRadToAngleDeg(MotorRad) + self.relEncOffsetAngleDeg

    def _angleDegToMotorRad(self, armAngleDeg: float) -> float:
        return math.radians((armAngleDeg - self.relEncOffsetAngleDeg) * ARM_GEARBOX_GEAR_RATIO )
    
    def _angleVelDegpsToMotorVelRadps(self, armAngleDeg: float) -> float:
        return armAngleDeg * ARM_GEARBOX_GEAR_RATIO

    def getRelAngleWithNoOffsetDeg(self) -> float:
        return self._noOffsetMotorRadToAngleDeg(self.motor.getMotorPositionRad())

    def getRelAngleWithOffsetDeg(self) -> float:
        return self._motorRadToAngleWithOffsetDeg(self.motor.getMotorPositionRad())

    def getVelocityDegps(self) -> float:
        return self._noOffsetMotorRadToAngleDeg(self.motor.getMotorVelocityRadPerSec())
    
    #return the angle of the arm as measured by the absolute sensor in angles
    def getAbsAngleDeg(self) -> float:
        angleAbsSenDeg = math.degrees(self.encoder.getAngleRad())
        return angleAbsSenDeg - ABS_SENSOR_MOUNT_OFFSET_DEG

    # def limitToMaxes(self, valueRad: float) -> float:
    #     if math.degrees(valueRad) >= 90:
    #         valueRad = math.radians(90)
    #     elif math.degrees(valueRad) <= -90:
    #         valueRad = math.radians(-90)
    #     else:
    #         valueRad = valueRad
    #     return valueRad

    # This routine uses the absolute sensors to adjust the offsets for the relative sensors
    # so that the relative sensors match reality.
    # It should be called.... infrequently. Likely once shortly after robot init.
    def getRelToAbsoluteSensorOffsetDeg(self) -> float:
        relAngleWithNoOffsetDeg = self.getRelAngleWithNoOffsetDeg()
        absAngleDeg = self.getAbsAngleDeg()

        relEncOffsetAngleDeg = absAngleDeg - relAngleWithNoOffsetDeg

        #if self.count % 1000 == 0:
        #    print(f"count={self.count} {relEncOffsetAngleDeg:+10.1f}=absAngleDeg({absAngleDeg:+10.1f} - relAngleWithNoOffsetDeg({relAngleWithNoOffsetDeg:+10.1f})\n... motorPositionRad={self.motor.getMotorPositionRad():+10.1f}->{self._noOffsetMotorRadToAngleDeg(self.motor.getMotorPositionRad()):+10.1f}")

        return relEncOffsetAngleDeg

    def update(self) -> None:
        self.encoder.update()

        #only need an uninitialized and an operating state
        #uninitialized - read the abs encoder angle and read the internal angle of the Spark Max to set 
        #offset that will be applied to all measurments so Spark Max knows current angle
        match self.state:
            case ArmStates.ARM_UNINITIALIZED:
                self._updateUninitialized()
            case ArmStates.ARM_OPERATING:
                self._updateOperating()
            case _:
                pass

        self.count = self.count + 1

    def _updateUninitialized(self) -> None:
        self.encoder.update()

        self.trapProfiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxVelocityDegps.get(), self.maxAccelerationDegps2.get()))
        self.lastStoppedTimeS = 0
        self.relEncOffsetAngleDeg = self.getRelToAbsoluteSensorOffsetDeg()
        self.curTrapPState = TrapezoidProfile.State(self.getRelAngleWithOffsetDeg(),0)

        self.state = ArmStates.ARM_OPERATING

    def _updateOperating(self) -> None:
        self.encoder.update()
        self.currentUpdateTimeS = Timer.getFPGATimestamp()
        self.actualVelDegps = self.getVelocityDegps()
        if self.previousUpdateTimeS is not None:
            currentPeriodS = self.currentUpdateTimeS - self.previousUpdateTimeS
            self.actAccLogger.logNow((self.actualVelDegps - self.previousVelDegps) / currentPeriodS)

        self.actTrapPState = TrapezoidProfile.State(self.getRelAngleWithOffsetDeg(), self.actualVelDegps)

        self.desTrapPState = TrapezoidProfile.State(self.armGoalDeg, 0)

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
            oldVelocityDegps = self.curTrapPState.velocity

            # this line does the main work of the profiler. It steps through
            # each of the target positions and velocities to get self.curTrapPState.position
            # through the set of positions you need to run the profile

            # search for self.trapProfiler initialization for the definition of the profile
            timeStepSeconds = 0.02
            self.curTrapPState = self.trapProfiler.calculate(timeStepSeconds, self.curTrapPState, self.desTrapPState)

            self.curTrapPAccLogger.logNow((self.curTrapPState.velocity - oldVelocityDegps) / 0.02)

            self.motorPosCmdRad = self._angleDegToMotorRad(self.curTrapPState.position)
            #self.motorPosCmdRad = self.limitToMaxes(self._angleDegToMotorRad(self.curTrapPState.position))
            motorVelCmdRadps = self._angleVelDegpsToMotorVelRadps(self.curTrapPState.velocity)

            # set our feed forward to 0 at the start so we're not throwing extra voltage into the motor, then see if their feed forward calc makes sense
            # vFF = self.kV.get() * motorVelCmdRadps  + self.kS.get() * sign(motorVelCmdRadps) \
            #    + self.kG.get()

            vFF = 0

            self.motor.setPosCmd(self.motorPosCmdRad, vFF)

        self.previousVelDegps = self.actualVelDegps
        self.previousUpdateTimeS = self.currentUpdateTimeS

        # API to set current height goal
    def setAngleGoal(self, armGoalDeg:float) -> None:
        self.armGoalDeg = armGoalDeg

    # API to confirm we are oK to be at an angle other than angle for pen and plunge
    def setSafeToLeavePenPosture(self, safe:bool) -> None:
        self.coralSafe = safe

    def setManualAdjCmd(self, cmd:float) -> None:
        self.manualAdjCmd = cmd

    def forceStartAtAngleZeroDeg(self) -> None:
        self.relEncOffsetRad = self.motor.getMotorPositionRad()
