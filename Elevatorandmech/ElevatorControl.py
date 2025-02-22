# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Elevator

# It is definitely buggy and untested, but it gives us a great framework on how to control an elevator.

from enum import IntEnum
from playingwithfusion import TimeOfFlight
from utils.calibration import Calibration
from utils.units import sign
from utils.signalLogging import  addLog
#from utils.constants import ELEV_LM_CANID, ELEV_RM_CANID, ELEV_TOF_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from rev import SparkLowLevel
from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer

ELEV_GEARBOX_GEAR_RATIO = 5.0/1.0
ELEV_SPOOL_RADIUS_IN = 1.92/2.0

MAX_ELEV_VEL_INPS = 20
MAX_ELEV_ACCEL_INPS2 = 4

REEF_L1_HEIGHT_M = 0.5842
REEF_L2_HEIGHT_M = 0.9398
REEF_L3_HEIGHT_M = 1.397 
REEF_L4_HEIGHT_M = 2.159 
ELEV_MIN_HEIGHT_M = REEF_L1_HEIGHT_M  # TODO - is elevator's bottom position actually L1?

ELEV_RM_CANID = 20
ELEV_LM_CANID = 21
ELEV_TOF_CANID = 22

class ElevatorStates(IntEnum):
    UNINITIALIZED = 0
    INIT_GOING_DOWN = 1
    FOUND_BOTTOM = 2
    OPERATING = 3
    NO_CMD = -1


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
        self.rMotor = WrapperedSparkMax(ELEV_RM_CANID, "ElevatorMotorRight", brakeMode=True, currentLimitA=10)
        self.lMotor = WrapperedSparkMax(ELEV_LM_CANID, "ElevatorMotorLeft", brakeMode=True, currentLimitA=10)
        self.lMotor.setFollow(ELEV_RM_CANID, True)


        # FF and proportional gain constants
        self.kV = Calibration(name="Elevator kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Elevator kS", default=0.1, units="V")
        self.kG = Calibration(name="Elevator kG", default=0.25, units="V")
        self.kP = Calibration(name="Elevator kP", default=0.05, units="V/rad error") # Per 0.001 seconds

        # Set P gain on motor
        self.rMotor.setPID(self.kP.get(), 0.0, 0.0)

        # Profiler
        self.maxV = Calibration(name="Elevator Max Vel", default=MAX_ELEV_VEL_INPS, units="inps")
        self.maxA = Calibration(name="Elevator Max Accel", default=MAX_ELEV_ACCEL_INPS2, units="inps2")
        self.trapProfiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxV.get(),self.maxA.get()))
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

        self.profiledPos = 0.0
        self.curUnprofiledPosCmd = 0.0

        self.elevatorState = ElevatorStates.UNINITIALIZED

        self.HeightGoalIn = 100000

        self.timeWhenChangeS = 0

        self.lowestHeightIn = 10000



        addLog("Elevator Goal Height", lambda: self.heightGoalIn, "in")
        addLog("Elevator Stopped", lambda: self.stopped, "bool")
        addLog("Elevator Profiled Height", lambda: self.curTrapPState.position, "in")
        addLog("Elevator State", lambda: float(int(self.elevatorState)), "int")

    def _RmotorRadToHeightIn(self, RmotorRad: float) -> float:
        #return RmotorRad * 1/ELEV_GEARBOX_GEAR_RATIO * (ELEV_SPOOL_RADIUS_M) - self.relEncOffsetM
        return  (RmotorRad-self.relEncOffsetRad) * (1/ELEV_GEARBOX_GEAR_RATIO) * ELEV_SPOOL_RADIUS_IN

    def _heightInToMotorRad(self, elevHeightIn: float) -> float:
        return (elevHeightIn / ELEV_SPOOL_RADIUS_IN) * ELEV_GEARBOX_GEAR_RATIO + self.relEncOffsetRad
    
    def _heightVelInpsToMotorVelRadps(self, elevVelInps: float) -> float:
        return (elevVelInps / ELEV_SPOOL_RADIUS_IN) * ELEV_GEARBOX_GEAR_RATIO
    
    def getHeightIn(self) -> float:
        return self._RmotorRadToHeightIn(self.rMotor.getMotorPositionRad())
    
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
        match self.elevatorState:
            case ElevatorStates.UNINITIALIZED:
                self._updateUninitialized()
            case ElevatorStates.INIT_GOING_DOWN:
                self._updateInitGoingDown()
            case ElevatorStates.FOUND_BOTTOM:
                self._updateFoundBottom()
            case ElevatorStates.OPERATING:
                self._updateOperating()
            case ElevatorStates.NO_CMD:
                pass
            case _:
                pass

    def _updateUninitialized(self) -> None:
        self.startTime = Timer.getFPGATimestamp()
        self._changeState(ElevatorStates.INIT_GOING_DOWN)
        positionRad = self.rMotor.getMotorPositionRad()
        goalPositionRad = positionRad - self._heightInToMotorRad(1000)
        self.desTrapPState = TrapezoidProfile.State(goalPositionRad,0)
        self._setMotorPosAndFF()
        self.lowestHeightIn = 100000

    def _updateInitGoingDown(self) -> None:
        # in going down, check to see if how long since we last moved, if we haven't moved in 1 seocond stop
            # make a new self.lowestHeightIn
            # set to 100,000 in when first start
            # make var of time since last change of height, use getFPGATimestamp whenever it gets lower to get the time
            # when lowest height hasn't changed for 1s (current time - last changed time >= 1s) then switch to found bottom state
        positionRad = self.rMotor.getMotorPositionRad()
        positionIn = self.getHeightIn()

        if positionIn < self.lowestHeightIn:
            self.lowestHeightIn = positionIn
            self.timeWhenChangeS = Timer.getFPGATimestamp()
            # change the time we last moved in seconds
        else:
            # because we didnt go any lower, maybe we have been at the lowest height for a second
            nowS = Timer.getFPGATimestamp()
            if nowS - 1 >= self.timeWhenChangeS:
                self._changeState(ElevatorStates.OPERATING)

        pass

    def _updateFoundBottom(self) -> None:
        self.bottom = self.getHeightIn()
        self._changeState(ElevatorStates.OPERATING)

    def _setMotorPosAndFF(self) -> None:
        self.curTrapPState = self.trapProfiler.calculate(0.02, self.curTrapPState, self.desTrapPState)

        motorPosCmdRad = self._heightInToMotorRad(self.curTrapPState.position)
        motorVelCmdRadps = self._heightVelInpsToMotorVelRadps(self.curTrapPState.velocity)

        # set our feed forward to 0 at the start so we're not throwing extra voltage into the motor
        # then see if their feed forward calc makes sense
        vFF = self.kV.get() * motorVelCmdRadps  + self.kS.get() * sign(motorVelCmdRadps) \
            + self.kG.get()

        vFF = 0

        self.rMotor.setPosCmd(motorPosCmdRad, vFF)

    def _updateOperating(self) -> None:
        self.actualPosIn = self.getHeightIn()

        self.desTrapPState = TrapezoidProfile.State(self.heightGoalIn,0)

        # Update motor closed-loop calibration
        if self.kP.isChanged():
            self.rMotor.setPID(self.kP.get(), 0.0, 0.0)

        if self.stopped:
            # Handle stopped by just holding mechanism in place with gravity offset, no closed loop.
            # TODO - do we need a more gentle stop here?
            manAdjVoltage = self.manAdjMaxVoltage.get() * self.manualAdjCmd

            self.rMotor.setVoltage(self.kG.get() + manAdjVoltage)
            self.curTrapPState = TrapezoidProfile.State(self.actualPosIn,0)
        else:
            self._setMotorPosAndFF()


    # API to set current height goal
    def setHeightGoal(self, heightGoalIn:float) -> None:
        self.heightGoalIn = heightGoalIn

    # API to confirm we are oK to be at a height other than L1
    def setSafeToLeaveL1(self, safe:bool) -> None:
        self.coralSafe = safe

    def setManualAdjCmd(self, cmd:float) -> None:
        self.manualAdjCmd = cmd

    def _changeState(self, newState: ElevatorStates) -> None:
        print(f"time = {Timer.getFPGATimestamp():.3f}s changing from elevator state {self.elevatorState} to {newState}")
        self.elevatorState = newState