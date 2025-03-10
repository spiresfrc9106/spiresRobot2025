# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Elevator

# It is definitely buggy and untested, but it gives us a great framework on how to control an elevator.

from enum import IntEnum
from wpilib import XboxController
from wpilib import Timer
from wpimath.trajectory import TrapezoidProfile

from utils.calibration import Calibration
from utils.robotIdentification import RobotIdentification
from utils.signalLogging import  addLog, getNowLogger
from utils.singleton import Singleton
from utils.units import sign
from utils.robotIdentification import RobotTypes
from wrappers.wrapperedSparkMax import WrapperedSparkMax

TEST_ELEVATOR_RANGE_IN = 5.2

class ElevatorDependentConstants:
    def __init__(self):
        self.elevDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_ELEVATOR": False,
                "ELEV_RM_CANID": None,
                "ELEV_LM_CANID": None,
                "ELEVATOR_RANGE_IN": None,
                "ELEV_GEARBOX_GEAR_RATIO": None,
                "ELEV_SPOOL_RADIUS_IN": None,
                "MAX_ELEV_VEL_INPS": None,
                "MAX_ELEV_ACCEL_INPS2": None,
            },
            RobotTypes.Spires2025: {
                "HAS_ELEVATOR": True,
                "ELEV_RM_CANID": 97,
                "ELEV_LM_CANID": 98,
                "ELEVATOR_RANGE_IN": 30, # xyzzy fix me up
                "ELEV_GEARBOX_GEAR_RATIO": 3.0 / 1.0,
                "ELEV_SPOOL_RADIUS_IN": 1.660 / 2.0,
                "MAX_ELEV_VEL_INPS": 5.0,  # TODO xyzzy - talk to Micahel Vu - must be a units issue
                "MAX_ELEV_ACCEL_INPS2": 5.0,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_ELEVATOR": True,
                "ELEV_RM_CANID": 20,
                "ELEV_LM_CANID": None,
                "ELEVATOR_RANGE_IN": 5.2,
                "ELEV_GEARBOX_GEAR_RATIO": 5.0 / 1.0,
                "ELEV_SPOOL_RADIUS_IN": 1.92 / 2.0,
                "MAX_ELEV_VEL_INPS": 25,
                "MAX_ELEV_ACCEL_INPS2": 250,
        },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_ELEVATOR": False,
                "ELEV_RM_CANID": None,
                "ELEV_LM_CANID": None,
                "ELEVATOR_RANGE_IN": None,
                "ELEV_GEARBOX_GEAR_RATIO": None,
                "ELEV_SPOOL_RADIUS_IN": None,
                "MAX_ELEV_VEL_INPS": None,
                "MAX_ELEV_ACCEL_INPS2": None,
            },
        }

    def get(self):
        return self.elevDepConstants[RobotIdentification().getRobotType()]

elevDepConstants = ElevatorDependentConstants().get()

ELEV_RM_CANID = elevDepConstants['ELEV_RM_CANID']
ELEV_LM_CANID = elevDepConstants['ELEV_LM_CANID']
ELEVATOR_RANGE_IN = elevDepConstants['ELEVATOR_RANGE_IN']

ELEV_GEARBOX_GEAR_RATIO = elevDepConstants['ELEV_GEARBOX_GEAR_RATIO']
ELEV_SPOOL_RADIUS_IN = elevDepConstants['ELEV_SPOOL_RADIUS_IN']

MAX_ELEV_VEL_INPS = elevDepConstants['MAX_ELEV_VEL_INPS']
MAX_ELEV_ACCEL_INPS2 = elevDepConstants['MAX_ELEV_ACCEL_INPS2']

REEF_L1_HEIGHT_M = 0.5842
REEF_L2_HEIGHT_M = 0.9398
REEF_L3_HEIGHT_M = 1.397
REEF_L4_HEIGHT_M = 2.159
ELEV_MIN_HEIGHT_M = REEF_L1_HEIGHT_M  # TODO - is elevator's bottom position actually L1?


class ElevatorStates(IntEnum):
    UNINITIALIZED = 0
    INIT_GOING_DOWN = 1
    OPERATING = 2
    NO_CMD = -1

TIME_STEP_S = 0.02

class ElevatorControl(metaclass=Singleton):
    def __init__(self):
        
        self.name = "elev"

        ctrlIdx = 1
        self.ctrl = XboxController(ctrlIdx)

        # Coral Scoring Heights in meters
        self.L1_Height = Calibration(name="Elevator Preset Height L1", units="m", default=REEF_L1_HEIGHT_M - ELEV_MIN_HEIGHT_M)
        self.L2_Height = Calibration(name="Elevator Preset Height L2", units="m", default=REEF_L2_HEIGHT_M - ELEV_MIN_HEIGHT_M)
        self.L3_Height = Calibration(name="Elevator Preset Height L3", units="m", default=REEF_L3_HEIGHT_M - ELEV_MIN_HEIGHT_M)
        self.L4_Height = Calibration(name="Elevator Preset Height L4", units="m", default=REEF_L4_HEIGHT_M - ELEV_MIN_HEIGHT_M)

        self.manAdjMaxVoltage = Calibration(name="Elevator Manual Adj Max Voltage", default=1.0, units="V")

        self.coralSafe = True
        self.manualAdjCmd = 0.0

        # See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
        self.desTrapPState = TrapezoidProfile.State(0,0)

        # Elevator Motors
        self.rMotor = WrapperedSparkMax(ELEV_RM_CANID, "ElevatorMotorRight", brakeMode=False, currentLimitA=5)
        rMotorIsInverted = True
        self.rMotor.setInverted(rMotorIsInverted)
        if ELEV_LM_CANID is not None:
            self.lMotor = WrapperedSparkMax(ELEV_LM_CANID, "ElevatorMotorLeft", brakeMode=False, currentLimitA=5)
            self.lMotor.setFollow(ELEV_RM_CANID, invert=True)
        else:
            self.lMotor = None


        # FF and proportional gain constants
        self.kV = Calibration(name="Elevator kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Elevator kS", default=0.1, units="V")
        self.kG = Calibration(name="Elevator kG", default=0.25, units="V")
        self.kP = Calibration(name="Elevator kP", default=0.4, units="V/rad error") # Per 0.001 seconds

        self.initialized = False

    def initialize(self):
        if not self.initialized:
            # Set P gain on motor
            self.rMotor.setPID(self.kP.get(), 0.0, 0.0)

            # Profiler
            self.maxVelocityIps = Calibration(name="Elevator Max Vel", default=MAX_ELEV_VEL_INPS, units="inps")
            self.maxAccelerationIps2 = Calibration(name="Elevator Max Accel", default=MAX_ELEV_ACCEL_INPS2, units="inps2")

            self.searchMaxVelocityIps = Calibration(name="Elevator Max Vel", default=4, units="inps")
            self.searchMaxAccelerationIps2 = Calibration(name="Elevator Max Accel", default=4, units="inps2")

            self.trapProfiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxVelocityIps.get(), self.maxAccelerationIps2.get()))
            self.actTrapPState = self.trapProfiler.State()
            self.curTrapPState = self.trapProfiler.State()

            self.stopped = False

            # Relative Encoder Offsets
            # Relative encoders always start at 0 at power-on
            # However, we may or may not have the mechanism at the "zero" position when we powered on
            # These variables store an offset which is calculated from the absolute sensors
            # to make sure the relative sensors inside the encoders accurately reflect
            # the actual position of the mechanism
            self.relEncOffsetRad = 0.0

            # Create a motion profile with the given maximum velocity and maximum
            # acceleration constraints for the next setpoint.

            self.profiledPos = 0.0
            self.curUnprofiledPosCmd = 0.0
            self.previousUpdateTimeS = None  # TOOD See if Michael needs this

            self.elevatorState = ElevatorStates.UNINITIALIZED

            self.timeWhenChangeS = 0

            self.lowestHeightIn = 1000

            self.funRuns = 0

            addLog(f"{self.name}/lowestHeightIn", lambda: self.lowestHeightIn, "in")
            addLog(f"{self.name}/state", lambda: float(int(self.elevatorState)), "int")
            addLog(f"{self.name}/stopped", lambda: self.stopped, "bool")
            addLog(f"{self.name}/act_pos_in", lambda: self.actTrapPState.position, "in")
            addLog(f"{self.name}/act_vel_inps", lambda: self.actTrapPState.velocity, "inps")
            self.actAccLogger = getNowLogger(f"{self.name}/act_acc_inps2", "inps2")
            addLog(f"{self.name}/curProfile_pos_in", lambda: self.curTrapPState.position, "in")
            addLog(f"{self.name}/curProfile_vel_inps", lambda: self.curTrapPState.velocity, "inps")
            self.curTrapPAccLogger = getNowLogger(f"{self.name}/curProfile_acc_inps2", "inps2")
            addLog(f"{self.name}/des_pos_in", lambda: self.desTrapPState.position, "in")
            addLog(f"{self.name}/des_vel_inps", lambda: self.desTrapPState.velocity, "inps")

            self.initialized = True

            print(f"Init {self.name} complete")

    def disable(self):
        self.rMotor.setPosCmd(self.rMotor.getMotorPositionRad())
        self.rMotor.setVoltage(0)

    def _offsetFreeRMotorRadToHeightIn(self, rMotorRad: float) -> float:
        return  rMotorRad * (1/ELEV_GEARBOX_GEAR_RATIO) * ELEV_SPOOL_RADIUS_IN

    def _rMotorRadToHeightIn(self, rMotorRad: float) -> float:
        return  self._offsetFreeRMotorRadToHeightIn(rMotorRad-self.relEncOffsetRad)

    def _heightInToMotorRad(self, elevHeightIn: float) -> float:
        return (elevHeightIn / ELEV_SPOOL_RADIUS_IN) * ELEV_GEARBOX_GEAR_RATIO + self.relEncOffsetRad
    
    def _heightVelInpsToMotorVelRadps(self, elevVelInps: float) -> float:
        return (elevVelInps / ELEV_SPOOL_RADIUS_IN) * ELEV_GEARBOX_GEAR_RATIO
    
    def getHeightIn(self) -> float:
        return self._rMotorRadToHeightIn(self.rMotor.getMotorPositionRad())
    def getVelocityInps(self) -> float:
        return self._offsetFreeRMotorRadToHeightIn(self.rMotor.getMotorVelocityRadPerSec())


    def update(self) -> None:
        match self.elevatorState:
            case ElevatorStates.UNINITIALIZED:
                self._updateUninitialized()
            case ElevatorStates.INIT_GOING_DOWN:
                self._updateInitGoingDown()
            case ElevatorStates.OPERATING:
                self._updateOperating()
            case ElevatorStates.NO_CMD:
                pass
            case _:
                pass

    def _updateUninitialized(self) -> None:
        self.startTime = Timer.getFPGATimestamp()
        self._changeState(ElevatorStates.INIT_GOING_DOWN)
        self.forceStartAtHeightZeroIn()
        self.desTrapPState = TrapezoidProfile.State(-1000,0)
        self.curTrapPState = TrapezoidProfile.State(self.getHeightIn(), 0)
        self._setMotorPosAndFF()
        self.lowestHeightIn = 1000

    def _updateInitGoingDown(self) -> None:
        self.funRuns = 0
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
            self._setMotorPosAndFF()
            # change the time we last moved in seconds
        else:
            # because we didnt go any lower, maybe we have been at the lowest height for a second
            nowS = Timer.getFPGATimestamp()
            if nowS - 1 >= self.timeWhenChangeS:
                self._changeState(ElevatorStates.OPERATING)
            else:
                self._setMotorPosAndFF()

    def _setMotorPosAndFF(self) -> None:
        oldVelocityInps = self.curTrapPState.velocity

        # This method is called both when initializing the elevator in the uninitialized state and when operating
        # In the initializing state we always have a target velocity at the end. In the operating state we might
        # have a target velocity that is not zero. If the target velocity is not zero we limit the velocity to be a
        # velocity that won't overrun the end limits based upon our current profiled position and the max acceleration.
        # Rather than solving with calculus, we just run an extra profiler and select the result with the smallest
        # magnitude of velocity.

        # TODO make sure all of the self. that we reference here are actually set before we touch

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

        self.curTrapPAccLogger.logNow((self.curTrapPState.velocity - oldVelocityInps) / TIME_STEP_S)

        motorPosCmdRad = self._heightInToMotorRad(self.curTrapPState.position)
        motorVelCmdRadps = self._heightVelInpsToMotorVelRadps(self.curTrapPState.velocity)

        # set our feed forward to 0 at the start so we're not throwing extra voltage into the motor
        # then see if their feed forward calc makes sense
        vFF = self.kV.get() * motorVelCmdRadps  + self.kS.get() * sign(motorVelCmdRadps) \
            + self.kG.get()

        vFF = 0

        self.rMotor.setPosCmd(motorPosCmdRad, vFF)

    def _perhapsWeHaveANewRangeCheckedDesiredState(self, newDesHeightIn, newDesVelocityInps):
        if (self.elevatorState == ElevatorStates.OPERATING) and (self.desTrapPState.position != newDesHeightIn or self.desTrapPState.velocity != newDesHeightIn):
            # limit the height goal so that it is less than max height
            # limit the height goal so that is more than 0
            newDesHeightIn = min(newDesHeightIn, TEST_ELEVATOR_RANGE_IN)
            newDesHeightIn = max(newDesHeightIn, self.lowestHeightIn)

            newDesVelocityInps = min(newDesVelocityInps, self.maxVelocityIps.get())
            newDesVelocityInps = max(newDesVelocityInps, -self.maxVelocityIps.get())

            # do these checks relative to the curTrapPState
            # if height goal is to go up, make sure that velocity goal is 0 or +
            if newDesHeightIn > self.curTrapPState.position:
                newDesVelocityInps = max(0, newDesVelocityInps)

            # if height goals is to go down, make sure that the velocity goal is 0 or -
            elif newDesHeightIn < self.curTrapPState.position:
                newDesVelocityInps = min(0, newDesVelocityInps)
            # if height goals is to stay at current height goal
            else:
                newDesVelocityInps = 0

            self.desTrapPState = self.trapProfiler.State(newDesHeightIn, newDesVelocityInps)

    def _updateOperating(self) -> None:
        if self.funRuns == 0:
            self.curTrapPState = TrapezoidProfile.State(self.getHeightIn(), 0)
        self.funRuns = self.funRuns + 1
        self.currentUpdateTimeS = Timer.getFPGATimestamp()
        self.actualVelInps = self.getVelocityInps()
        if self.previousUpdateTimeS is not None:
            currentPeriodS = self.currentUpdateTimeS - self.previousUpdateTimeS
            self.actAccLogger.logNow((self.actualVelInps - self.previousVelInps) / currentPeriodS)

        # The default is to go to the middle
        self.actTrapPState = TrapezoidProfile.State(self.getHeightIn(), self.actualVelInps)
        self.desTrapPState = TrapezoidProfile.State(self.lowestHeightIn + (TEST_ELEVATOR_RANGE_IN / 2),0)

        if self.ctrl.getAButton():
            self.desTrapPState = TrapezoidProfile.State(REEF_L1_HEIGHT_M * 1 + self.lowestHeightIn, 0)
        if self.ctrl.getXButton():
            self.desTrapPState = TrapezoidProfile.State(REEF_L2_HEIGHT_M * 1 + self.lowestHeightIn, 0)
        if self.ctrl.getBButton():
            self.desTrapPState = TrapezoidProfile.State(REEF_L3_HEIGHT_M * 1 + self.lowestHeightIn, 0)
        if self.ctrl.getYButton():
            self.desTrapPState = TrapezoidProfile.State(REEF_L4_HEIGHT_M * 1 + self.lowestHeightIn, 0)

        # PLUNGE USING THE RIGHT TRIGGER
        if self.ctrl.getRightTriggerAxis():
            self.desTrapPState = TrapezoidProfile.State(self.lowestHeightIn, 0)

        self._setMotorPosAndFF()

        # Update motor closed-loop calibration
        if self.kP.isChanged():
            self.rMotor.setPID(self.kP.get(), 0.0, 0.0)

        if self.stopped:
            # Handle stopped by just holding mechanism in place with gravity offset, no closed loop.
            # TODO - do we need a more gentle stop here?
            manAdjVoltage = self.manAdjMaxVoltage.get() * self.manualAdjCmd

            self.rMotor.setVoltage(self.kG.get() + manAdjVoltage)
            self.curTrapPState = TrapezoidProfile.State(self.actTrapPState.position,0)
        else:
            self._setMotorPosAndFF()

        self.previousVelInps = self.actualVelInps
        self.previousUpdateTimeS = self.currentUpdateTimeS

    # API to set current height goal
    def setHeightGoal(self, heightGoalIn:float) -> None:
        self._perhapsWeHaveANewRangeCheckedDesiredState(newDesHeightIn=heightGoalIn, newDesVelocityInps = 0.0)

    # todo make an new API function:
    def setHeightVelocityGoal(self, heightGoalIn:float, velocityGoalInps:float) -> None:
        self._perhapsWeHaveANewRangeCheckedDesiredState(newDesHeightIn=heightGoalIn, newDesVelocityInps=velocityGoalInps)


    # API to confirm we are oK to be at a height other than L1
    def setSafeToLeaveL1(self, safe:bool) -> None:
        self.coralSafe = safe

    def setManualAdjCmd(self, cmd:float) -> None:
        self.manualAdjCmd = cmd

    def forceStartAtHeightZeroIn(self) -> None:
        self.relEncOffsetRad = self.rMotor.getMotorPositionRad()

    def _changeState(self, newState: ElevatorStates) -> None:
        print(f"time = {Timer.getFPGATimestamp():.3f}s changing from elevator state {self.elevatorState} to {newState}")
        self.elevatorState = newState
