# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Arm

# It is definitely buggy and untested, but it gives us a great framework on how to control an elevator.

#Notes to self:
#1) abs encoder needs to have constant calibration offset applied to it, and calibrated offset is then applied to rel
#2) setting up state machine so that it will be iterated through; make sure states are changed!

from enum import Enum
from playingwithfusion import TimeOfFlight

from Elevatorandmech.ArmConstants import ARM_GEARBOX_GEAR_RATIO
from utils.calibration import Calibration
from utils.units import sign
from utils.signalLogging import  addLog, getNowLogger
#from utils.constants import ARM_LM_CANID, ARM_RM_CANID, ARM_TOF_CANID
from utils.singleton import Singleton
from wrappers.wrapperedPulseWidthEncoder import WrapperedPulseWidthEncoder
from wrappers.wrapperedRevThroughBoreEncoder import WrapperedRevThroughBoreEncoder
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from rev import SparkLowLevel
from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer
import math


MAX_ARM_VEL_DEGPS = 20 # Could be 80
MAX_ARM_ACCEL_DEGPS2 = 4 # Could be 160

ARM_M_CANID = 20

#will need to use abs encoder angle to find offset for 
#Spark Max angle, then Spark Max angle can be used
class ArmStates(Enum):
    ARM_UNINITIALIZED = 0
    ARM_OPERATING = 2


class ArmControl(metaclass=Singleton):
    def __init__(self):
        #there will not be preset angles for heights,
        #it will just be going to the angle given by Noah's code
        

        self.manAdjMaxVoltage = Calibration(name="Arm Manual Adj Max Voltage", default=1.0, units="V")

        self.armGoalDeg = 0.0
        
        # See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
        self.desTrapPState = TrapezoidProfile.State(self.armGoalDeg,0)

        # Arm Motors
        self.Motor = WrapperedSparkMax(ARM_M_CANID, "ArmMotor", brakeMode=False, currentLimitA=5)
        MotorIsInverted = True
        self.Motor.setInverted(MotorIsInverted)

        # Rev Relative Encoder
        self.Encoder = WrapperedRevThroughBoreEncoder(port=9, name="ArmRevRelEncoder", mountOffsetRad=0, dirInverted=False)
        self.angleAbsSen = self.Encoder.getAngleRad()

        # FF and proportional gain constants
        self.kV = Calibration(name="Arm kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Arm kS", default=0.1, units="V")
        self.kG = Calibration(name="Arm kG", default=0.25, units="V")
        self.kP = Calibration(name="Arm kP", default=0.4, units="V/rad error") # Per 0.001 seconds

        # Set P gain on motor
        self.Motor.setPID(self.kP.get(), 0.0, 0.0)

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


        # This variable stores the offset for the Abs sensor to define what angle is
        # zero based on the Abs sensor's physical orientation on the robot
        self.ABS_SENSOR_CALIBRATION_OFFSET_DEG = 0 # TODO find this

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

        self.state = ArmStates.ARM_UNINITIALIZED

        addLog("Arm Goal Degree", lambda: self.armGoalDeg, "deg")
        addLog("Arm Stopped", lambda: self.stopped, "bool")
        addLog("Arm act Degree", lambda: self.actTrapPState.position, "deg")
        addLog("Arm act Velocity", lambda: self.actTrapPState.velocity, "degps")
        self.actAccLogger = getNowLogger("Arm act Acceleration", "degps2")
        addLog("Arm curProfile Degree", lambda: self.curTrapPState.position, "deg")
        addLog("Arm curProfile Velocity", lambda: self.curTrapPState.velocity, "degps")
        self.curTrapPAccLogger = getNowLogger("Arm curProfile Acceleration", "degps2")
        addLog("Arm Desired Degree", lambda: self.desTrapPState.position, "deg")
        addLog("Arm Desired Velocity", lambda: self.desTrapPState.velocity, "degps")
        addLog("Arm Degree Error", lambda: self.actTrapPState.position-self.curTrapPState.position, "deg")
        addLog("Arm Velocity Error", lambda: self.actTrapPState.velocity-self.curTrapPState.velocity, "deg")

        self.profiledPos = 0.0
        self.curUnprofiledPosCmd = 0.0
        self.previousUpdateTimeS = None
        self.previousVelDegps = None

    def _angleInRange(self, angle: float) -> float:
        angle%= 360
        angle = (angle + 360) % 360
        if (angle > 180): angle -= 360
        return angle

    def _offsetFreeMotorRadToAngleDeg(self, MotorRad: float) -> float:
        return  (math.degrees(MotorRad) * (1/ARM_GEARBOX_GEAR_RATIO)) - self.relEncOffsetAngleDeg

    def _MotorRadToAngleDeg(self, MotorRad: float) -> float:
        return  self._offsetFreeMotorRadToAngleDeg(MotorRad)

    def _angleDegToMotorRad(self, armAngleDeg: float) -> float:
        return armAngleDeg * ARM_GEARBOX_GEAR_RATIO + self.relEncOffsetRad
    
    def _angleVelDegpsToMotorVelRadps(self, armAngleDeg: float) -> float:
        return armAngleDeg * ARM_GEARBOX_GEAR_RATIO

    def getRelAngleDeg(self) -> float:
        return self._angleInRange(self._MotorRadToAngleDeg(self.Motor.getMotorPositionRad()))

    def getVelocityDegps(self) -> float:
        return self._offsetFreeMotorRadToAngleDeg(self.Motor.getMotorVelocityRadPerSec())
    
    #return the angle of the arm as measured by the absolute sensor in angles
    def _getAbsAngle(self) -> float:
        self.angleAbsSen = (math.degrees(self.angleAbsSen))
        return (self._angleInRange(self.angleAbsSen)) - self.ABS_SENSOR_CALIBRATION_OFFSET_DEG

    # This routine uses the absolute sensors to adjust the offsets for the relative sensors
    # so that the relative sensors match reality.
    # It should be called.... infrequently. Likely once shortly after robot init.
    def initFromAbsoluteSensor(self) -> None:
        # Reset offsets to zero, so the relative sensor get functions return
        # just whatever offset the relative sensor currently has.
        self.relEncOffsetAngleDeg = 0.0

        # New Offset = real angle - what angle says??
        self.relEncOffsetAngleDeg = self._getAbsAngle() - self.getRelAngleDeg()


    def update(self) -> None:
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

    def _updateUninitialized(self) -> None:
        self.trapProfiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxVelocityDegps.get(), self.maxAccelerationDegps2.get()))
        self.lastStoppedTimeS = 0
        self.lowestAngleDeg = 1000.0

        # going to try and use abs sensor to provide offset for rel encoder, not what is commented below
        #self.forceStartAtAngleZeroDeg()
        self.initFromAbsoluteSensor()
        motorPosCmdRad = self._angleDegToMotorRad(0)
        motorVelCmdRadps = self._angleVelDegpsToMotorVelRadps(0)

        # set our feed forward to 0 at the start so we're not throwing extra voltage into the motor, then see if their feed forward calc makes sense
        # vFF = self.kV.get() * motorVelCmdRadps  + self.kS.get() * sign(motorVelCmdRadps) \
        #    + self.kG.get()

        vFF = 0

        self.Motor.setPosCmd(motorPosCmdRad, vFF)
        self.state = ArmStates.ARM_OPERATING

    def _updateOperating(self) -> None:
        self.currentUpdateTimeS = Timer.getFPGATimestamp()
        self.actualVelDegps = self.getVelocityDegps()
        if self.previousUpdateTimeS is not None:
            currentPeriodS = self.currentUpdateTimeS - self.previousUpdateTimeS
            self.actAccLogger.logNow((self.actualVelDegps - self.previousVelDegps) / currentPeriodS)

        self.actTrapPState = TrapezoidProfile.State(self.getRelAngleDeg(), self.actualVelDegps)

        self.desTrapPState = TrapezoidProfile.State(self.armGoalDeg,0)

        # Update motor closed-loop calibration
        if self.kP.isChanged():
            self.Motor.setPID(self.kP.get(), 0.0, 0.0)

        #this doesn't happen right now
        if self.stopped:
            # Handle stopped by just holding mechanism in place with gravity offset, no closed loop.
            # TODO - do we need a more gentle stop here?
            manAdjVoltage = self.manAdjMaxVoltage.get() * self.manualAdjCmd

            self.Motor.setVoltage(self.kG.get() + manAdjVoltage)
            self.curTrapPState = TrapezoidProfile.State(self.actTrapPState.position, 0)
        else:
            #this case does happen
            oldVelocityDegps = self.curTrapPState.velocity
            
            #this line does the main work of the profiler. It steps through
            #each of the target positions and velocities to get self.curTrapPState.position
            #through the set of positions you need to run the profile
            
            #search for self.trapProfiler initialization for the definition of the profile
            timeStepSeconds = 0.02
            self.curTrapPState = self.trapProfiler.calculate(timeStepSeconds, self.curTrapPState, self.desTrapPState)
            
            self.curTrapPAccLogger.logNow((self.curTrapPState.velocity - oldVelocityDegps) / 0.02)

            motorPosCmdRad = self._angleDegToMotorRad(self.curTrapPState.position)
            motorVelCmdRadps = self._angleVelDegpsToMotorVelRadps(self.curTrapPState.velocity)

            # set our feed forward to 0 at the start so we're not throwing extra voltage into the motor, then see if their feed forward calc makes sense
            #vFF = self.kV.get() * motorVelCmdRadps  + self.kS.get() * sign(motorVelCmdRadps) \
            #    + self.kG.get()

            vFF = 0

            self.Motor.setPosCmd(motorPosCmdRad, vFF)

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
        self.relEncOffsetRad = self.Motor.getMotorPositionRad()
