# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Elevator

# It is definitely buggy and untested, but it gives us a great framework on how to control an elevator.

from enum import Enum
from playingwithfusion import TimeOfFlight
from utils.calibration import Calibration
from utils.units import sign
from utils.signalLogging import  addLog, getNowLogger
#from utils.constants import ELEV_LM_CANID, ELEV_RM_CANID, ELEV_TOF_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from rev import SparkLowLevel
from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer

ELEV_GEARBOX_GEAR_RATIO = 5.0/1.0
ELEV_SPOOL_RADIUS_IN = 1.92/2.0

MAX_ELEV_VEL_INPS = 20 # Could be 80
MAX_ELEV_ACCEL_INPS2 = 4 # Could be 160

REEF_L1_HEIGHT_M = 0.5842
REEF_L2_HEIGHT_M = 0.9398
REEF_L3_HEIGHT_M = 1.397 
REEF_L4_HEIGHT_M = 2.159 
ELEV_MIN_HEIGHT_M = REEF_L1_HEIGHT_M  # TODO - is elevator's bottom position actually L1?

ELEV_RM_CANID = 20
ELEV_LM_CANID = 21
ELEV_TOF_CANID = 22

class ElevatorStates(Enum):
    ELEVATOR_INITIALIZING = 0
    ELEVATOR_STOPPED = 1
    ELEVATOR_CMD = 2
    ELEVATOR_NO_CMD = -1


class ElevatorControl(metaclass=Singleton):
    def __init__(self):

        # Coral Scoring Heights in meters
        self.L1_Height = Calibration(name="Elevator Preset Height L1", units="m", default=REEF_L1_HEIGHT_M - ELEV_MIN_HEIGHT_M)
        self.L2_Height = Calibration(name="Elevator Preset Height L2", units="m", default=REEF_L2_HEIGHT_M - ELEV_MIN_HEIGHT_M)
        self.L3_Height = Calibration(name="Elevator Preset Height L3", units="m", default=REEF_L3_HEIGHT_M - ELEV_MIN_HEIGHT_M)
        self.L4_Height = Calibration(name="Elevator Preset Height L4", units="m", default=REEF_L4_HEIGHT_M - ELEV_MIN_HEIGHT_M)

        self.manAdjMaxVoltage = Calibration(name="Elevator Manual Adj Max Voltage", default=1.0, units="V")

        self.heightGoalIn = 0.0
        self.coralSafe = True
        self.manualAdjCmd = 0.0

        # See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
        self.desTrapPState = TrapezoidProfile.State(self.heightGoalIn,0)

        # Elevator Motors
        self.Rmotor = WrapperedSparkMax(ELEV_RM_CANID, "ElevatorMotorRight", brakeMode=False, currentLimitA=20)
        rMotorIsInverted = True
        self.Rmotor.setInverted(rMotorIsInverted)
        self.LMotor = WrapperedSparkMax(ELEV_LM_CANID, "ElevatorMotorLeft", brakeMode=False, currentLimitA=20)
        self.LMotor.setFollow(ELEV_RM_CANID, True)


        # FF and proportional gain constants
        self.kV = Calibration(name="Elevator kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Elevator kS", default=0.1, units="V")
        self.kG = Calibration(name="Elevator kG", default=0.25, units="V")
        self.kP = Calibration(name="Elevator kP", default=0.4, units="V/rad error") # Per 0.001 seconds
        #self.kP = Calibration(name="Elevator kP", default=0.05, units="V/rad error") # Per 0.001 seconds

        # Set P gain on motor
        self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)

        # Profiler
        self.maxVelocityIps = Calibration(name="Elevator Max Vel", default=MAX_ELEV_VEL_INPS, units="inps")
        self.maxAccelerationIps2 = Calibration(name="Elevator Max Accel", default=MAX_ELEV_ACCEL_INPS2, units="inps2")
        self.trapProfiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxVelocityIps.get(), self.maxAccelerationIps2.get()))
        self.actTrapPState = self.trapProfiler.State()
        self.curTrapPState = self.trapProfiler.State()
        #go to wpilib online documentation to learn more about the trapezoid (very cool)

        self.actualPosIn = 0
        self.stopped = False

        # Try to set a small current limit and decide when we're on the bottom using this, and turn off the motor when it doesn't need to spin anymore.  then up the current limit as needed


        # Absolute Sensor mount offsets: use laser range finders to measure distance? either sensor or revolution count

        # After mounting the sensor, these should be tweaked one time
        # in order to adjust whatever the sensor reads into the reference frame
        # of the mechanism
        self.ABS_SENSOR_READING_AT_ELEVATOR_BOTTOM_M = 0.074 # TODO correct?

        # Relative Encoder Offsets
        # Releative encoders always start at 0 at power-on
        # However, we may or may not have the mechanism at the "zero" position when we powered on
        # These variables store an offset which is calculated from the absolute sensors
        # to make sure the relative sensors inside the encoders accurately reflect
        # the actual position of the mechanism
        self.relEncOffsetRad = 0.0
        # Create a motion profile with the given maximum velocity and maximum
        # acceleration constraints for the next setpoint.

        self.heightGoalIn = 0.0

        addLog("Elevator Goal Height", lambda: self.heightGoalIn, "in")
        addLog("Elevator Stopped", lambda: self.stopped, "bool")
        addLog("Elevator actPosIn", lambda: self.actualPosIn, "in")
        addLog("Elevator act Height", lambda: self.actTrapPState.position, "in")
        addLog("Elevator act Velocity", lambda: self.actTrapPState.velocity, "inps")
        addLog("Elevator curProfile Height", lambda: self.curTrapPState.position, "in")
        addLog("Elevator curProfile Velocity", lambda: self.curTrapPState.velocity, "inps")
        addLog("Elevator Desired Height", lambda: self.desTrapPState.position, "in")
        addLog("Elevator Desired Velocity", lambda: self.desTrapPState.velocity, "inps")
        self.actAccLogger = getNowLogger("Elevator act Acceleration", "inps2")
        self.curTrapPAccLogger = getNowLogger("Elevator curProfile Acceleration", "inps2")
        addLog("Elevator Height Error", lambda: self.actTrapPState.position-self.curTrapPState.position, "in")
        addLog("Elevator Velocity Error", lambda: self.actTrapPState.velocity-self.curTrapPState.velocity, "in")


        self.profiledPos = 0.0
        self.curUnprofiledPosCmd = 0.0
        self.previousUpdateTimeS = None

    def _offsetFreeRmotorRadToHeightIn(self, RmotorRad: float) -> float:
        #return RmotorRad * 1/ELEV_GEARBOX_GEAR_RATIO * (ELEV_SPOOL_RADIUS_M) - self.relEncOffsetM
        return  RmotorRad * (1/ELEV_GEARBOX_GEAR_RATIO) * ELEV_SPOOL_RADIUS_IN

    def _RmotorRadToHeightIn(self, RmotorRad: float) -> float:
        return  self._offsetFreeRmotorRadToHeightIn(RmotorRad-self.relEncOffsetRad)

    def _heightInToMotorRad(self, elevHeightIn: float) -> float:
        return (elevHeightIn / ELEV_SPOOL_RADIUS_IN) * ELEV_GEARBOX_GEAR_RATIO + self.relEncOffsetRad
    
    def _heightVelInpsToMotorVelRadps(self, elevVelInps: float) -> float:
        return (elevVelInps / ELEV_SPOOL_RADIUS_IN) * ELEV_GEARBOX_GEAR_RATIO

    
    def getHeightIn(self) -> float:
        return self._RmotorRadToHeightIn(self.Rmotor.getExternalAbsoluteEncoderRad()) # TODO xyzzy change this to the internal position rad


    def getVelocityInps(self) -> float:
        return self._offsetFreeRmotorRadToHeightIn(self.Rmotor.getExternalAbsoluteEncoderVelocityRadPerSec())
    
    #return the height of the elevator as measured by the absolute sensor in meters
    def _getAbsHeight(self) -> float:
        return self.heightAbsSen.getRange() / 1000.0 - self.ABS_SENSOR_READING_AT_ELEVATOR_BOTTOM_M

    # This routine uses the absolute sensors to adjust the offsets for the relative sensors
    # so that the relative sensors match reality.
    # It should be called.... infrequently. Likely once shortly after robot init.
    def initFromAbsoluteSensor(self) -> None:
        # Reset offsets to zero, so the relative sensor get functions return
        # just whatever offset the relative sensor currently has.
        self.relEncOffsetM = 0.0

        # New Offset = real height - what height says?? 
        self.relEncOffsetM = self._getAbsHeight() - self.getHeightIn()

    def update(self) -> None:
        self.currentUpdateTimeS = Timer.getFPGATimestamp()
        self.actualVelInps = self.getVelocityInps()
        if self.previousUpdateTimeS is not None:
            currentPeriodS = self.currentUpdateTimeS - self.previousUpdateTimeS
            self.actAccLogger.logNow((self.actualVelInps - self.previousVelInps) / currentPeriodS)

        self.actualPosIn = self.getHeightIn()

        self.actTrapPState = TrapezoidProfile.State(self.actualPosIn, self.actualVelInps)

        self.desTrapPState = TrapezoidProfile.State(self.heightGoalIn,0)

        # Update motor closed-loop calibration
        if self.kP.isChanged():
            self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)

        if self.stopped:
            # Handle stopped by just holding mechanism in place with gravity offset, no closed loop.
            # TODO - do we need a more gentle stop here?
            manAdjVoltage = self.manAdjMaxVoltage.get() * self.manualAdjCmd

            self.Rmotor.setVoltage(self.kG.get() + manAdjVoltage)
            self.curTrapPState = TrapezoidProfile.State(self.actualPosIn, 0)
        else:
            oldVelocityInps = self.curTrapPState.velocity
            self.curTrapPState = self.trapProfiler.calculate(0.02, self.curTrapPState, self.desTrapPState)

            self.curTrapPAccLogger.logNow((self.curTrapPState.velocity - oldVelocityInps) / 0.02)

            motorPosCmdRad = self._heightInToMotorRad(self.curTrapPState.position)
            motorVelCmdRadps = self._heightVelInpsToMotorVelRadps(self.curTrapPState.velocity)

            # set our feed forward to 0 at the start so we're not throwing extra voltage into the motor, then see if their feed forward calc makes sense
            #vFF = self.kV.get() * motorVelCmdRadps  + self.kS.get() * sign(motorVelCmdRadps) \
            #    + self.kG.get()

            vFF = 0

            self.Rmotor.setPosCmd(motorPosCmdRad, vFF)

        self.previousVelInps = self.actualVelInps
        self.previousUpdateTimeS = self.currentUpdateTimeS

        # API to set current height goal
    def setHeightGoal(self, heightGoalIn:float) -> None:
        self.heightGoalIn = heightGoalIn

    # API to confirm we are oK to be at a height other than L1
    def setSafeToLeaveL1(self, safe:bool) -> None:
        self.coralSafe = safe

    def setManualAdjCmd(self, cmd:float) -> None:
        self.manualAdjCmd = cmd

    def forceStartAtHeightZeroIn(self) -> None:
        self.relEncOffsetRad = self.Rmotor.getExternalAbsoluteEncoderRad()
