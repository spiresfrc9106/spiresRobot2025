import time
from rev import SparkMax, SparkBase, SparkMaxConfig, REVLibError, ClosedLoopSlot, SparkBaseConfig
from rev import SparkClosedLoopController
from wpilib import TimedRobot
from utils.signalLogging import addLog
from utils.units import rev2Rad, rad2Rev, radPerSec2RPM, RPM2RadPerSec
from utils.faults import Fault
from wrappers.wrapperedSparkCommon import MotorControlStates


## Wrappered Spark Max
# Wrappers REV's libraries to add the following functionality for spark max controllers:
# Grouped PID controller, Encoder, and motor controller objects
# Physical unit conversions into SI units (radians)
# Retry logic for initial configuration
# Fault handling for not crashing code if the motor controller is disconnected
# Fault annunication logic to trigger warnings if a motor couldn't be configured
class WrapperedSparkMax:
    def __init__(self, canID, name, brakeMode=False, currentLimitA=40):
        self.ctrl = SparkMax(canID, SparkMax.MotorType.kBrushless)
        self.closedLoopCtrl = self.ctrl.getClosedLoopController()
        self.encoder = self.ctrl.getEncoder()
        self.name = name
        self.currentLimitA = round(currentLimitA)
        self.configSuccess = False
        self.disconFault = Fault(f"Spark Max {name} ID {canID} disconnected")
        self.simActPos = 0
        self.canID = canID

        # pylint: disable= R0801
        self.desPosRad = 0
        self.desVelRadps = 0
        self.desVolt = 0
        self.actPosRad = 0
        self.actVelRadps = 0
        self.actVolt = 0
        self.controlState = MotorControlStates.UNKNOWN

        self.cfg = SparkMaxConfig()
        self.cfg.signals.appliedOutputPeriodMs(200)
        self.cfg.signals.busVoltagePeriodMs(200)
        self.cfg.signals.primaryEncoderPositionPeriodMs(20)
        self.cfg.signals.primaryEncoderVelocityPeriodMs(200)
        self.cfg.setIdleMode(SparkBaseConfig.IdleMode.kBrake if brakeMode else SparkBaseConfig.IdleMode.kCoast)
        self.cfg.smartCurrentLimit(self.currentLimitA,0,5700)

        self._sparkmax_config(retries=10, resetMode=SparkBase.ResetMode.kResetSafeParameters, persistMode=SparkBase.PersistMode.kPersistParameters, step="Initial Config")

        addLog(self.name + "_outputCurrent", self.ctrl.getOutputCurrent, "A")
        addLog(self.name + "_desVolt", lambda: self.desVolt, "V")
        addLog(self.name + "_desPos", lambda: self.desPosRad, "rad")
        addLog(self.name + "_desVel", lambda: self.desVelRadps, "radps")
        addLog(self.name + "_actVolt", lambda: self.actVolt, "V")
        addLog(self.name + "_actPos", lambda: self.actPosRad, "rad")
        addLog(self.name + "_actVel", lambda: RPM2RadPerSec(self.encoder.getVelocity()), "radps")
        print(f"Init of SparkMax {self.name} CANID={self.canID} is finished")

    def _sparkmax_config(self, retries, resetMode, persistMode, printResults=True, step=""):
        # Perform motor configuration, tracking errors and retrying until we have success
        # Clear previous configuration, and persist anything set in this config.
        retryCounter = 0
        success=False
        while not success and retryCounter < retries:
            retryCounter += 1
            err = self.ctrl.configure(self.cfg, resetMode, persistMode)

            # Check if any operation triggered an error
            if err != REVLibError.kOk:
                if printResults:
                    print(
                        f"{step} Failure configuring Spark Max {self.name} CAN ID {self.canID}, retrying..."
                    )
            else:
                # Only attempt other communication if we're able to successfully configure
                if printResults:
                    print(f"{step} Successfully connected to {self.name} motor")
                success = True
            if retryCounter < retries:
                time.sleep(0.1)

        self.configSuccess = success

        self.disconFault.set(not self.configSuccess)

    def setFollow(self, leaderCanID, invert=False):
        self.cfg.follow(leaderCanID, invert)
        self.ctrl.configure(self.cfg,
                                SparkBase.ResetMode.kNoResetSafeParameters, 
                                SparkBase.PersistMode.kPersistParameters)

    def setInverted(self, isInverted):
        if self.configSuccess:
            self.cfg.inverted(isInverted)
            self.ctrl.configure(self.cfg,
                                SparkBase.ResetMode.kNoResetSafeParameters, 
                                SparkBase.PersistMode.kPersistParameters)

    def setPID(self, kP, kI, kD, persist=SparkBase.PersistMode.kPersistParameters):
        if self.configSuccess:
            self.cfg.closedLoop.pid(kP, kI, kD, ClosedLoopSlot.kSlot0)
            # Apply new configuration
            # but don't reset other parameters
            # Use the specified persist mode.
            # By default we persist setings (usually we set PID once, then don't think about it again)
            # However, if setPID is getting called in a periodic loop, don't bother persisting the parameters
            # because the persist operation takes a long time on the spark max.
            self.ctrl.configure(self.cfg, 
                                SparkBase.ResetMode.kNoResetSafeParameters, 
                                persist)
            
    def setPosCmd(self, posCmd, arbFF=0.0):
        """_summary_

        Args:
            posCmd (float): motor desired shaft rotations in radians
            arbFF (int, optional): _description_. Defaults to 0.
        """
        self.simActPos = posCmd
        posCmdRev = rad2Rev(posCmd)

        self.desPosRad = posCmd
        self.desVolt = arbFF

        if self.configSuccess:
            err = self.closedLoopCtrl.setReference(
                posCmdRev,
                SparkMax.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                arbFF,
                SparkClosedLoopController.ArbFFUnits.kVoltage,
            )
            self.controlState = MotorControlStates.POSITION

            self.disconFault.set(err != REVLibError.kOk)



    def setVelCmd(self, velCmd, arbFF=0.0):
        """_summary_

        Args:
            velCmd (float): motor desired shaft velocity in radians per second
            arbFF (int, optional): _description_. Defaults to 0.
        """

        self.desVelRadps = velCmd
        desVelRPM = radPerSec2RPM(velCmd)
        self.desVolt = arbFF

        if self.configSuccess:
            err = self.closedLoopCtrl.setReference(
                desVelRPM,
                SparkMax.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                arbFF,
                SparkClosedLoopController.ArbFFUnits.kVoltage,
            )
            self.controlState = MotorControlStates.VELOCITY
            self.disconFault.set(err != REVLibError.kOk)

    def setVoltage(self, outputVoltageVolts):
        self.desVolt = outputVoltageVolts
        if self.configSuccess:
            self.ctrl.setVoltage(outputVoltageVolts)
            self.controlState = MotorControlStates.VOLTAGE

    def getMotorPositionRad(self):
        if(TimedRobot.isSimulation()):
            pos = self.simActPos
        else:
            if self.configSuccess:
                pos = rev2Rad(self.encoder.getPosition())
            else:
                pos = 0
        self.actPosRad = pos
        return pos

    def getMotorVelocityRadPerSec(self):
        if self.configSuccess:
            vel = self.encoder.getVelocity()
        else:
            vel = 0
        self.actVelRadps = RPM2RadPerSec(vel)
        return self.actVelRadps

    def getAppliedOutput(self):
        self.actVolt = self.ctrl.getAppliedOutput() * 12
        return self.actVolt

    def getCurrentLimitA(self)->int:
        return self.currentLimitA

    def getControlState(self)->MotorControlStates:
        return self.controlState

    def setSmartCurrentLimit(self, currentLimitA: int)->None:
        self.currentLimitA = round(currentLimitA)
        self.cfg.smartCurrentLimit(self.currentLimitA,0,5700)
        self._sparkmax_config(retries=4, resetMode=SparkBase.ResetMode.kNoResetSafeParameters, persistMode=SparkBase.PersistMode.kNoPersistParameters, printResults=True, step="Current Limit")

    def getOutputCurrentA(self)->float:
        return self.ctrl.getOutputCurrent()
