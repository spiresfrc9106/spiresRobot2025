# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Elevator

# It is definitely buggy and untested, but it gives us a great framework on how to control an elevator.

from enum import IntEnum
import math
from wpimath.trajectory import TrapezoidProfile
import wpilib
from wpilib import Timer

from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.RobotPoser import PoseDirector
from utils.calibration import Calibration
from utils.robotIdentification import RobotIdentification
from utils.signalLogging import  addLog, getNowLogger
from utils.singleton import Singleton
from utils.units import sign
from utils.robotIdentification import RobotTypes
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.motorStallDetector import MotorPosStallDetector

class ElevatorDependentConstants:
    def __init__(self):

        self.elevDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_ELEVATOR": False,
                "ELEV_FM_CANID": None,
                "ELEV_BM_CANID": None,
                "ELEV_M_INITIALIZING_CURRENT_LIMIT_A": 20,
                "ELEV_M_OPERATING_CURRENT_LIMIT_A": 20,
                "ELEVATOR_RANGE_IN": None,
                "ELEV_GEARBOX_GEAR_RATIO": None,
                "ELEV_SPOOL_RADIUS_IN": None,
                "MAX_ELEV_VEL_INPS": None,
                "MAX_ELEV_ACCEL_INPS2": None,
                "ELEVATOR_HEIGHT_OFFSET": None,
            },
            RobotTypes.Spires2025: {
                "HAS_ELEVATOR": True,
                "ELEV_FM_CANID": 12,
                "ELEV_BM_CANID": 16,
                "ELEV_M_INITIALIZING_CURRENT_LIMIT_A": 40,
                "ELEV_M_OPERATING_CURRENT_LIMIT_A": 60,
                "ELEVATOR_RANGE_IN": 46,  # reverted to 46 from 47 bc we tightened rope
                "ELEV_GEARBOX_GEAR_RATIO": 3.0 / 1.0,
                "ELEV_SPOOL_RADIUS_IN": 1.660 / 2.0,
                "MAX_ELEV_VEL_INPS": 60.0,
                "MAX_ELEV_ACCEL_INPS2": 120.0,
                "ELEVATOR_HEIGHT_OFFSET": 15.25,
            },
            RobotTypes.Spires2025Sim: {
                "HAS_ELEVATOR": True,
                "ELEV_FM_CANID": 28,
                "ELEV_BM_CANID": 29,
                "ELEV_M_INITIALIZING_CURRENT_LIMIT_A": 20,
                "ELEV_M_OPERATING_CURRENT_LIMIT_A": 60,
                "ELEVATOR_RANGE_IN": 46,
                "ELEV_GEARBOX_GEAR_RATIO": 3.0 / 1.0,
                "ELEV_SPOOL_RADIUS_IN": 1.660 / 2.0,
                "MAX_ELEV_VEL_INPS": 20.0,
                "MAX_ELEV_ACCEL_INPS2": 20.0,
                "ELEVATOR_HEIGHT_OFFSET": 15.25,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_ELEVATOR": True,
                "ELEV_FM_CANID": 20,
                "ELEV_BM_CANID": None,
                "ELEV_M_INITIALIZING_CURRENT_LIMIT_A": 20,
                "ELEV_M_OPERATING_CURRENT_LIMIT_A": 20,
                "ELEVATOR_RANGE_IN": 5.2,
                "ELEV_GEARBOX_GEAR_RATIO": 5.0 / 1.0,
                "ELEV_SPOOL_RADIUS_IN": 1.92 / 2.0,
                "MAX_ELEV_VEL_INPS": 25,
                "MAX_ELEV_ACCEL_INPS2": 250,
                "ELEVATOR_HEIGHT_OFFSET": 0,
            },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_ELEVATOR": False,
                "ELEV_FM_CANID": None,
                "ELEV_BM_CANID": None,
                "ELEV_M_INITIALIZING_CURRENT_LIMIT_A": 20,
                "ELEV_M_OPERATING_CURRENT_LIMIT_A": 20,
                "ELEVATOR_RANGE_IN": None,
                "ELEV_GEARBOX_GEAR_RATIO": None,
                "ELEV_SPOOL_RADIUS_IN": None,
                "MAX_ELEV_VEL_INPS": None,
                "MAX_ELEV_ACCEL_INPS2": None,
                "ELEVATOR_HEIGHT_OFFSET": None,
            },
        }

    def get(self):
        return self.elevDepConstants[RobotIdentification().getRobotType()]

elevDepConstants = ElevatorDependentConstants().get()

ELEV_FM_CANID = elevDepConstants['ELEV_FM_CANID']
ELEV_BM_CANID = elevDepConstants['ELEV_BM_CANID']

ELEV_M_INITIALIZING_CURRENT_LIMIT_A = elevDepConstants['ELEV_M_INITIALIZING_CURRENT_LIMIT_A'] # xyzzy CAUTION we're not using this yet
ELEV_M_OPERATING_CURRENT_LIMIT_A = elevDepConstants['ELEV_M_OPERATING_CURRENT_LIMIT_A']
ELEV_GEARBOX_GEAR_RATIO = elevDepConstants['ELEV_GEARBOX_GEAR_RATIO']
ELEV_SPOOL_RADIUS_IN = elevDepConstants['ELEV_SPOOL_RADIUS_IN']

MAX_ELEV_VEL_INPS = elevDepConstants['MAX_ELEV_VEL_INPS']
MAX_ELEV_ACCEL_INPS2 = elevDepConstants['MAX_ELEV_ACCEL_INPS2']

ELEVATOR_RANGE_IN = elevDepConstants['ELEVATOR_RANGE_IN']

ELEV_HEIGHT_ABOVE_GROUND_IN = elevDepConstants['ELEVATOR_HEIGHT_OFFSET']

class ElevatorStates(IntEnum):
    UNINITIALIZED = 0
    INIT_GOING_DOWN = 1
    OPERATING = 2
    NO_CMD = -1

TIME_STEP_S = 0.02


class ElevatorControl(metaclass=Singleton):
    def __init__(self):
        
        self.name = "elev"

        self.state = ElevatorStates.UNINITIALIZED

        # please do not delete this xyzzy ask Yavin about this
        self.atAboutDown = False

        self.poseDirector = PoseDirector()

        # FF and proportional gain constants
        self.kV = Calibration(name="Elevator kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Elevator kS", default=0.1, units="V")
        self.kG = Calibration(name="Elevator kG", default=0.25, units="V")
        self.kP = Calibration(name="Elevator kP", default=0.4, units="V/rad error") # Per 0.001 seconds
        self.calElevMinHeightIn = Calibration(name="Elevator Min Height Above Ground In", default=ELEV_HEIGHT_ABOVE_GROUND_IN, units="in")
        self.calElevRangeIn = Calibration(name="Elevator Range In", default=ELEVATOR_RANGE_IN, units="in")
        self.calMaxVelocityInps = Calibration(name="Elevator Max Vel", default=MAX_ELEV_VEL_INPS, units="inps")
        self.calMaxAccelerationInps2 = Calibration(name="Elevator Max Accel", default=MAX_ELEV_ACCEL_INPS2, units="inps2")

        self.calSearchMaxVelocityInps = Calibration(name="Elevator Search Max Vel", default=10, units="inps")
        self.calSearchMaxAccelerationInps2 = Calibration(name="Elevator Search Max Accel", default=10, units="inps2")

        self.calElevInitializingCurrentLimitA = Calibration(name="Elevator INITIALIZING_CURRENT_LIMIT_A", default=ELEV_M_INITIALIZING_CURRENT_LIMIT_A, units="A")
        self.calElevOperatingCurrentLimitA = Calibration(name="Elevator OPERATING_CURRENT_LIMIT_A", default=ELEV_M_OPERATING_CURRENT_LIMIT_A, units="A")


        self.stateNowLogger = getNowLogger(f"{self.name}/stateNow", int)
        self.stateNowLogger.logNow(ElevatorStates.UNINITIALIZED)
        self.poserCmdPosLogger = getNowLogger(f"{self.name}/cmd_pos_in", "in")
        self.poserCmdPosLogger.logNow(100)
        self.poserCmdVelLogger = getNowLogger(f"{self.name}/cmd_vel_inps", "inps")
        self.poserCmdVelLogger.logNow(0)

        # See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
        self.desTrapPState = TrapezoidProfile.State(0,0) # Height 0 in, Velocity 0 in per s

        # Elevator Motors
        self.fMotor = WrapperedSparkMax(ELEV_FM_CANID, f"{self.name}/fMotor", brakeMode=True, currentLimitA=self.calElevInitializingCurrentLimitA.get())
        fMotorIsInverted = False
        self.fMotor.setInverted(fMotorIsInverted)
        if ELEV_BM_CANID is not None:
            self.bMotor = WrapperedSparkMax(ELEV_BM_CANID, f"{self.name}/bMotor", brakeMode=True, currentLimitA=self.calElevInitializingCurrentLimitA.get())
            self.bMotor.setFollow(ELEV_FM_CANID, invert=not fMotorIsInverted)
        else:
            self.bMotor = None

        self.stallDectector = MotorPosStallDetector(f"{self.name}/stall" ,self.fMotor, stallCurrentLimitA=self.fMotor.currentLimitA, stallTimeLimitS=0.2)

        self._initialized = False


    def initialize(self):
        changed = self.calElevMinHeightIn.isChanged() or \
                  self.calElevRangeIn.isChanged() or \
                  self.kP.isChanged() or \
                  self.calMaxVelocityInps.isChanged() or \
                  self.calMaxAccelerationInps2.isChanged() or \
                  self.calSearchMaxVelocityInps.isChanged() or \
                  self.calSearchMaxAccelerationInps2.isChanged() or \
                  self.calElevInitializingCurrentLimitA.isChanged() or \
                  self.calElevOperatingCurrentLimitA.isChanged()

        if changed or not self._initialized:
            # Set P gain on motor
            self.fMotor.setPID(self.kP.get(), 0.0, 0.0)
            # when we re-initialize tell motor to stay where it is at.
            self.fMotor.setVoltage(0)

            self.minHeightIn = self.calElevMinHeightIn.get()
            self.maxHeightIn = self.minHeightIn + self.calElevRangeIn.get()

            self.trapProfiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.calSearchMaxVelocityInps.get(), self.calSearchMaxAccelerationInps2.get()))
            self.actTrapPState = self.trapProfiler.State()
            self.curTrapPState = self.trapProfiler.State()

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

            self.lowestHeightIn = 0

            addLog(f"{self.name}/lowestHeightIn", lambda: self.lowestHeightIn, "in")
            addLog(f"{self.name}/state", lambda: self.state, "int")
            addLog(f"{self.name}/act_pos_in", lambda: self.actTrapPState.position, "in")
            addLog(f"{self.name}/act_vel_inps", lambda: self.actTrapPState.velocity, "inps")
            self.actAccLogger = getNowLogger(f"{self.name}/act_acc_inps2", "inps2")
            addLog(f"{self.name}/curProfile_pos_in", lambda: self.curTrapPState.position, "in")
            addLog(f"{self.name}/curProfile_vel_inps", lambda: self.curTrapPState.velocity, "inps")
            self.curTrapPAccLogger = getNowLogger(f"{self.name}/curProfile_acc_inps2", "inps2")
            addLog(f"{self.name}/des_pos_in", lambda: self.desTrapPState.position, "in")
            addLog(f"{self.name}/des_vel_inps", lambda: self.desTrapPState.velocity, "inps")

            addLog("RPelev/pos", lambda: self.actTrapPState.position, "in")

            self._changeState(ElevatorStates.UNINITIALIZED)

            self.previousUpdateTimeS = None
            self.previousVelDegps = None

            self.count = 0

            self._initialized = True

            print(f"Init {self.name} complete")

    def disable(self):
        self.fMotor.setPosCmd(self.fMotor.getMotorPositionRad())
        self.fMotor.setVoltage(0)

    def _offsetFreeRMotorRadToHeightIn(self, motorRad: float) -> float:
        return  motorRad * (1/ELEV_GEARBOX_GEAR_RATIO) * ELEV_SPOOL_RADIUS_IN + self.minHeightIn

    def _motorRadToHeightIn(self, motorRad: float) -> float:
        return  self._offsetFreeRMotorRadToHeightIn(motorRad+self.relEncOffsetRad)

    def _heightInToMotorRad(self, elevHeightIn: float) -> float:
        return ( (elevHeightIn - self.minHeightIn) / ELEV_SPOOL_RADIUS_IN) * ELEV_GEARBOX_GEAR_RATIO - self.relEncOffsetRad
    
    def _heightVelInpsToMotorVelRadps(self, elevVelInps: float) -> float:
        return (elevVelInps / ELEV_SPOOL_RADIUS_IN) * ELEV_GEARBOX_GEAR_RATIO
    
    def getHeightIn(self) -> float:
        return self._motorRadToHeightIn(self.fMotor.getMotorPositionRad())

    def _getVelocityInps(self) -> float:
        return self._offsetFreeRMotorRadToHeightIn(self.fMotor.getMotorVelocityRadPerSec())


    def update(self) -> None:
        self.fMotor.getMotorPositionRad()
        self.stallDectector.update()
        match self.state:
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
        self.timeWhenChangeS = Timer.getFPGATimestamp()
        self._changeState(ElevatorStates.INIT_GOING_DOWN)
        self._forceStartAtHeightZeroIn()
        if wpilib.RobotBase.isSimulation():
            self.desTrapPState = TrapezoidProfile.State(self.getHeightIn(),0)
        else:
            self.desTrapPState = TrapezoidProfile.State(self.getHeightIn()-1000,0)
        self.curTrapPState = TrapezoidProfile.State(self.getHeightIn(), 0)
        self.lowestHeightIn = self.getHeightIn()
        self._setMotorPosAndFF()

    def _updateInitGoingDown(self) -> None:
        positionIn = self.getHeightIn()

        if positionIn < self.lowestHeightIn:
            self.lowestHeightIn = positionIn
            self.timeWhenChangeS = Timer.getFPGATimestamp()
            self._setMotorPosAndFF()
            # change the time we last moved in seconds
        else:
            # because we didn't go any lower, maybe we have been at the lowest height for a second
            nowS = Timer.getFPGATimestamp()
            if nowS - 1 >= self.timeWhenChangeS:
                self._forceStartAtHeightZeroIn()
                self.trapProfiler = TrapezoidProfile(
                    TrapezoidProfile.Constraints(self.calMaxVelocityInps.get(),
                                                 self.calMaxAccelerationInps2.get()))
                self.desTrapPState = TrapezoidProfile.State(self.getHeightIn(), 0)
                self.curTrapPState = TrapezoidProfile.State(self.getHeightIn(), 0)
                self._changeState(ElevatorStates.OPERATING)
            else:
                self._setMotorPosAndFF()

    def _setMotorPosAndFF(self, enablePosMove=True) -> None:
        oldVelocityInps = self.curTrapPState.velocity

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

        self.curTrapPAccLogger.logNow((self.curTrapPState.velocity - oldVelocityInps) / TIME_STEP_S)

        motorPosCmdRad = self._heightInToMotorRad(self.curTrapPState.position)
        motorVelCmdRadps = self._heightVelInpsToMotorVelRadps(self.curTrapPState.velocity)

        # set our feed forward to 0 at the start so we're not throwing extra voltage into the motor
        # then see if their feed forward calc makes sense
        vFF = self.kV.get() * motorVelCmdRadps  + self.kS.get() * sign(motorVelCmdRadps) \
            + self.kG.get()

        vFF = 0

        if enablePosMove:
            self.fMotor.setPosCmd(motorPosCmdRad, vFF)
        else:
            self.fMotor.setVoltage(0)

    def _perhapsWeHaveANewRangeCheckedDesiredState(self, elevatorCommand: ElevatorCommand )->TrapezoidProfile.State:

        newDesHeightIn = elevatorCommand.heightIn
        newDesVelocityInps = elevatorCommand.velocityInps

        result = self.desTrapPState

        if (self.state == ElevatorStates.OPERATING) and (self.desTrapPState.position != newDesHeightIn or self.desTrapPState.velocity != newDesVelocityInps):

            newDesVelocityInps = min(newDesVelocityInps, self.calMaxVelocityInps.get())
            newDesVelocityInps = max(newDesVelocityInps, -self.calMaxVelocityInps.get())

            if newDesHeightIn is not None:

                # limit the height goal so that it is less than max height
                # limit the height goal so that is more than 0

                newDesHeightIn = min(newDesHeightIn, self.maxHeightIn)
                newDesHeightIn = max(newDesHeightIn, self.minHeightIn)

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
            elif newDesVelocityInps == 0.0:
                newDesHeightIn = self.curTrapPState.position
            elif newDesVelocityInps > 0.0:
                newDesHeightIn = self.maxHeightIn
            elif newDesVelocityInps < 0.0:
                newDesHeightIn = self.minHeightIn
            else:
                newDesHeightIn = newDesHeightIn

            result = self.trapProfiler.State(newDesHeightIn, newDesVelocityInps)

        return result

    def _updateOperating(self) -> None:
        self.currentUpdateTimeS = Timer.getFPGATimestamp()
        self.actualVelInps = self._getVelocityInps()
        if self.previousUpdateTimeS is not None:
            currentPeriodS = self.currentUpdateTimeS - self.previousUpdateTimeS
            self.actAccLogger.logNow((self.actualVelInps - self.previousVelInps) / currentPeriodS)

        self.actTrapPState = TrapezoidProfile.State(self.getHeightIn(), self.actualVelInps)

        # default to not moving
        heightGoalIn = self.curTrapPState.position
        velocityGoalInps = 0.0

        elevatorCommand = ElevatorCommand(heightGoalIn, velocityGoalInps)
        elevatorCommand = self.poseDirector.getElevatorCommand(elevatorCommand)

        if elevatorCommand.heightIn is None:
            self.poserCmdPosLogger.logNow(100)
        else:
            self.poserCmdPosLogger.logNow(elevatorCommand.heightIn)
        self.poserCmdVelLogger.logNow(elevatorCommand.velocityInps)


        self.desTrapPState = self._perhapsWeHaveANewRangeCheckedDesiredState(elevatorCommand)

        # Update motor closed-loop calibration
        if self.kP.isChanged():
            self.fMotor.setPID(self.kP.get(), 0.0, 0.0)


        self._setMotorPosAndFF()

        self.previousVelInps = self.actualVelInps
        self.previousUpdateTimeS = self.currentUpdateTimeS



    def _forceStartAtHeightZeroIn(self) -> None:
        self.relEncOffsetRad = 0.0 - self.fMotor.getMotorPositionRad()

    def _changeState(self, newState: ElevatorStates) -> None:
        print(f"time = {Timer.getFPGATimestamp():.3f}s changing from elevator state {self.state} to {newState.name}({newState})")
        self.state = newState
        self.stateNowLogger.logNow(self.state)
    
    # Yavin todo use these:  
    def getCurProfilePosIn(self):
        return self.curTrapPState.position
    
    def getCurProfileVelocityInps(self):
        return self.curTrapPState.velocity

    def getPosition(self):
        return self.getCurProfilePosIn()

    def getVelocity(self):
        return self.getCurProfileVelocityInps()