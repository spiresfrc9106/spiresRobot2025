from enum import IntEnum
import wpilib
from wpilib import Timer
from wpimath.trajectory import TrapezoidProfile

from Elevatorandmech.ElevatorCommand import ElevatorCommand
from Elevatorandmech.RobotPoser import PoseDirector
from utils.calibration import Calibration
from utils.robotIdentification import RobotIdentification
from utils.signalLogging import  addLog, getNowLogger
from utils.singleton import Singleton
from utils.units import sign
from utils.robotIdentification import RobotTypes
from wrappers.wrapperedSparkMax import WrapperedSparkMax

ELEV_HEIGHT_ABOVE_GROUND_IN = 15.25

class ElevatorDependentConstants:
    def __init__(self):

        self.elevDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_ELEVATOR": False,
                "ELEV_FM_CANID": None,
                "ELEV_BM_CANID": None,
                "ELEVATOR_RANGE_IN": None,
                "ELEV_GEARBOX_GEAR_RATIO": None,
                "ELEV_SPOOL_RADIUS_IN": None,
                "MAX_ELEV_VEL_INPS": None,
                "MAX_ELEV_ACCEL_INPS2": None,
            },
            RobotTypes.Spires2025: {
                "HAS_ELEVATOR": True,
                "ELEV_FM_CANID": 12,
                "ELEV_BM_CANID": 16,
                "ELEVATOR_RANGE_IN": 46, #was 47
                "ELEV_GEARBOX_GEAR_RATIO": 3.0 / 1.0,
                "ELEV_SPOOL_RADIUS_IN": 1.660 / 2.0,
                "MAX_ELEV_VEL_INPS": 20.0,
                "MAX_ELEV_ACCEL_INPS2": 20.0,
            },
            RobotTypes.Spires2025Sim: {
                "HAS_ELEVATOR": True,
                "ELEV_FM_CANID": 28,
                "ELEV_BM_CANID": 29,
                "ELEVATOR_RANGE_IN": 46,
                "ELEV_GEARBOX_GEAR_RATIO": 3.0 / 1.0,
                "ELEV_SPOOL_RADIUS_IN": 1.660 / 2.0,
                "MAX_ELEV_VEL_INPS": 20.0,
                "MAX_ELEV_ACCEL_INPS2": 20.0,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_ELEVATOR": True,
                "ELEV_FM_CANID": 20,
                "ELEV_BM_CANID": None,
                "ELEVATOR_RANGE_IN": 5.2,
                "ELEV_GEARBOX_GEAR_RATIO": 5.0 / 1.0,
                "ELEV_SPOOL_RADIUS_IN": 1.92 / 2.0,
                "MAX_ELEV_VEL_INPS": 25,
                "MAX_ELEV_ACCEL_INPS2": 250,
        },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_ELEVATOR": False,
                "ELEV_FM_CANID": None,
                "ELEV_BM_CANID": None,
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

ELEV_FM_CANID = elevDepConstants['ELEV_FM_CANID']
ELEV_BM_CANID = elevDepConstants['ELEV_BM_CANID']

ELEV_GEARBOX_GEAR_RATIO = elevDepConstants['ELEV_GEARBOX_GEAR_RATIO']
ELEV_SPOOL_RADIUS_IN = elevDepConstants['ELEV_SPOOL_RADIUS_IN']

MAX_ELEV_VEL_INPS = elevDepConstants['MAX_ELEV_VEL_INPS']
MAX_ELEV_ACCEL_INPS2 = elevDepConstants['MAX_ELEV_ACCEL_INPS2']

ELEVATOR_RANGE_IN = elevDepConstants['ELEVATOR_RANGE_IN']


class ElevatorStates(IntEnum):
    UNINITIALIZED = 0
    INIT_GOING_DOWN = 1
    OPERATING = 2
    NO_CMD = -1

TIME_STEP_S = 0.02

class ElevatorControl(metaclass=Singleton):
    def __init__(self):
        
        self.name = "elev"

        # please don't delete this - xyzzy TODO Mike ask Yavin about this.
        self.atAboutMiddle = True
        self.reachedBottom = False #these only need to be estimates, since the system will continue nmw

        self.poseDirector = PoseDirector()

        self.manAdjMaxVoltage = Calibration(name="Elevator Manual Adj Max Voltage", default=1.0, units="V")

        self.coralSafe = True
        self.manualAdjCmd = 0.0

        # See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
        self.desTrapPState = TrapezoidProfile.State(0,0) # Height 0 in, Velocity 0 in per s

        # Elevator Motors
        self.fMotor = WrapperedSparkMax(ELEV_FM_CANID, f"{self.name}/fMotor", brakeMode=True, currentLimitA=40)
        fMotorIsInverted = False
        self.fMotor.setInverted(fMotorIsInverted)
        if ELEV_BM_CANID is not None:
            self.bMotor = WrapperedSparkMax(ELEV_BM_CANID, f"{self.name}/bMotor", brakeMode=True, currentLimitA=40)
            self.bMotor.setFollow(ELEV_FM_CANID, invert=not fMotorIsInverted)
        else:
            self.bMotor = None


        # FF and proportional gain constants
        self.kV = Calibration(name="Elevator kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Elevator kS", default=0.1, units="V")
        self.kG = Calibration(name="Elevator kG", default=0.25, units="V")
        self.kP = Calibration(name="Elevator kP", default=0.4, units="V/rad error") # Per 0.001 seconds

        self.initialized = False

    def initialize(self):
        if not self.initialized:
            # Set P gain on motor
            self.fMotor.setPID(self.kP.get(), 0.0, 0.0)
            # when we re-initialize tell motor to stay where it is at.
            self.fMotor.setVoltage(0)

            # Profiler
            self.maxVelocityIps = Calibration(name="Elevator Max Vel", default=MAX_ELEV_VEL_INPS, units="inps")
            self.maxAccelerationIps2 = Calibration(name="Elevator Max Accel", default=MAX_ELEV_ACCEL_INPS2, units="inps2")

            self.searchMaxVelocityIps = Calibration(name="Elevator Search Max Vel", default=4, units="inps")
            self.searchMaxAccelerationIps2 = Calibration(name="Elevator Search Max Accel", default=4, units="inps2")

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

            self.lowestHeightIn = 0

            self.minPosIn = 0.0
            self.maxPosIn = ELEVATOR_RANGE_IN

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
        self.fMotor.setPosCmd(self.fMotor.getMotorPositionRad())
        self.fMotor.setVoltage(0)

    def _offsetFreeRMotorRadToHeightIn(self, motorRad: float) -> float:
        return  motorRad * (1/ELEV_GEARBOX_GEAR_RATIO) * ELEV_SPOOL_RADIUS_IN

    def _motorRadToHeightIn(self, motorRad: float) -> float:
        return  self._offsetFreeRMotorRadToHeightIn(motorRad-self.relEncOffsetRad)

    def _heightInToMotorRad(self, elevHeightIn: float) -> float:
        return (elevHeightIn / ELEV_SPOOL_RADIUS_IN) * ELEV_GEARBOX_GEAR_RATIO + self.relEncOffsetRad
    
    def _heightVelInpsToMotorVelRadps(self, elevVelInps: float) -> float:
        return (elevVelInps / ELEV_SPOOL_RADIUS_IN) * ELEV_GEARBOX_GEAR_RATIO
    
    def getHeightIn(self) -> float:
        return self._motorRadToHeightIn(self.fMotor.getMotorPositionRad())

    def getVelocityInps(self) -> float:
        return self._offsetFreeRMotorRadToHeightIn(self.fMotor.getMotorVelocityRadPerSec())

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
                self.forceStartAtHeightZeroIn()
                self.desTrapPState = TrapezoidProfile.State(self.getHeightIn(), 0)
                self.curTrapPState = TrapezoidProfile.State(self.getHeightIn(), 0)
                self._changeState(ElevatorStates.OPERATING)
            else:
                self._setMotorPosAndFF()

    def _setMotorPosAndFF(self) -> None:
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

        self.fMotor.setPosCmd(motorPosCmdRad, vFF)

    def _perhapsWeHaveANewRangeCheckedDesiredState(self, newDesHeightIn, newDesVelocityInps):
        if (self.elevatorState == ElevatorStates.OPERATING) and (self.desTrapPState.position != newDesHeightIn or self.desTrapPState.velocity != newDesVelocityInps):
            # limit the height goal so that it is less than max height
            # limit the height goal so that is more than 0

            a = newDesHeightIn
            b = newDesVelocityInps

            newDesHeightIn = min(newDesHeightIn, self.maxPosIn)
            newDesHeightIn = max(newDesHeightIn, self.minPosIn)

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

            print(f" a={a} b={b} newDesHeightIn={newDesHeightIn} newDesVelocityInps={newDesVelocityInps}")

            self.desTrapPState = self.trapProfiler.State(newDesHeightIn, newDesVelocityInps)

    def _updateOperating(self) -> None:
        self.currentUpdateTimeS = Timer.getFPGATimestamp()
        self.actualVelInps = self.getVelocityInps()
        if self.previousUpdateTimeS is not None:
            currentPeriodS = self.currentUpdateTimeS - self.previousUpdateTimeS
            self.actAccLogger.logNow((self.actualVelInps - self.previousVelInps) / currentPeriodS)

        self.actTrapPState = TrapezoidProfile.State(self.getHeightIn(), self.actualVelInps)

        # default to not moving
        heightGoalIn = self.curTrapPState.position
        velocityGoalInps = 0.0

        elevatorCommand = ElevatorCommand(heightGoalIn, velocityGoalInps)
        elevatorCommand = self.poseDirector.getElevatorCommand(elevatorCommand)


        #print(f"elevator {elevatorCommand.heightIn} {elevatorCommand.velocityInps}")
        if elevatorCommand.heightIn is None and elevatorCommand.velocityInps == 0.0:
            elevatorCommand.heightIn = heightGoalIn
        elif elevatorCommand.heightIn is None and elevatorCommand.velocityInps > 0.0:
            elevatorCommand.heightIn = self.maxPosIn
        else:
            elevatorCommand.heightIn = self.minPosIn

        self.desTrapPState = TrapezoidProfile.State(elevatorCommand.heightIn, elevatorCommand.velocityInps)

        # Update motor closed-loop calibration
        if self.kP.isChanged():
            self.fMotor.setPID(self.kP.get(), 0.0, 0.0)

        if self.stopped:
            # Handle stopped by just holding mechanism in place with gravity offset, no closed loop.
            # TODO - do we need a more gentle stop here?
            manAdjVoltage = self.manAdjMaxVoltage.get() * self.manualAdjCmd

            self.fMotor.setVoltage(self.kG.get() + manAdjVoltage)
            self.curTrapPState = TrapezoidProfile.State(self.actTrapPState.position,0)
        else:
            self._setMotorPosAndFF()

        self.previousVelInps = self.actualVelInps
        self.previousUpdateTimeS = self.currentUpdateTimeS

    # API to set current height goal
    def setHeightGoal(self, heightGoalIn:float) -> None:
        self._perhapsWeHaveANewRangeCheckedDesiredState(newDesHeightIn=heightGoalIn, newDesVelocityInps = 0.0)

    def setHeightVelocityGoal(self, heightGoalIn:float, velocityGoalInps:float) -> None:
        self._perhapsWeHaveANewRangeCheckedDesiredState(newDesHeightIn=heightGoalIn, newDesVelocityInps=velocityGoalInps)


    def setManualAdjCmd(self, cmd:float) -> None:
        self.manualAdjCmd = cmd

    def forceStartAtHeightZeroIn(self) -> None:
        self.relEncOffsetRad = self.fMotor.getMotorPositionRad()

    def _changeState(self, newState: ElevatorStates) -> None:
        print(f"time = {Timer.getFPGATimestamp():.3f}s changing from elevator state {self.elevatorState.name}({self.elevatorState}) to {newState.name}({newState})")
        self.elevatorState = newState

    def isOperating(self):
        return self.elevatorState == ElevatorStates.OPERATING

    def getElevatorPosIn(self):
        return self.actTrapPState.position
