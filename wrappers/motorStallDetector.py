




from wpilib import Timer

from utils.signalLogging import  addLog
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedSparkFlex import WrapperedSparkFlex
from wrappers.wrapperedSparkCommon import MotorControlStates


class MotorPosStallDetector:
    def __init__(self, name:str,
                 motor: WrapperedSparkMax|WrapperedSparkFlex,
                 stallCurrentLimitA: int,
                 stallTimeLimitS: float):
        self.name = name
        self.motor = motor
        self.stallCurrentLimitA = stallCurrentLimitA
        self.stallTimeLimitS = stallTimeLimitS
        self._stalled = False
        addLog(f"{name}/stalled", lambda: self._stalled)

    def initialize(self)->None:
        self._clearMonitorState()


    def _clearMonitorState(self)->None:
        self.lastMoveTimeS = None
        self.currentUpdateTimeS = None
        self.lastPositionRad = None
        self.lastMotorPosRad = None
        self.lastDirectionSign = None
        self.currentMotorPosRad = None
        self.currentMotorDirectionSign = None
        self.lastPositionErrorRad = None
        self.minPositionErr = None

    def _calcDirectionSign(self)->int:
        self.currentMotorPosRad = self.motor.getMotorPositionRad()
        if self.motor.desPosRad > self.currentMotorPosRad:
            return 1
        elif self.motor.desPosRad < self.currentMotorPosRad:
            return -1
        else:
            return 0

    def _hasMotorMovedTowardsDesPos(self):
        result = False
        if self.lastMotorPosRad is not None:
            if self.currentMotorDirectionSign == 1:
                if self.currentMotorPosRad > self.lastMotorPosRad:
                    result = True

            elif self.currentMotorPosRad == -1:
                if self.currentMotorPosRad < self.lastMotorPosRad:
                    result = True
            else:
                pass
        #print(f"_hasMotorMovedTowardsDesPos = {Timer.getFPGATimestamp():.3f} lastMotorRadPos={self.lastMotorPosRad:.1f} dir={self.currentMotorDirectionSign} cur={self.currentMotorPosRad:.1f} res={result})")
        return result

    def _isMotorStalled(self):
        result = False
        self.currentMotorDirectionSign = self._calcDirectionSign()

        #print(f"_isMotorStalled = {Timer.getFPGATimestamp():.3f} lastMoveTimeS={self.lastMoveTimeS} lastMotorPosRad={self.lastMotorPosRad} lastDir={self.lastDirectionSign} curDir={self.currentMotorDirectionSign})")

        if self.lastMoveTimeS is not None \
            and self.lastMotorPosRad is not None \
            and self.lastDirectionSign is not None \
            and self.currentMotorDirectionSign == self.lastDirectionSign:
                if self._hasMotorMovedTowardsDesPos():
                    self.lastMoveTimeS = self.currentUpdateTimeS
                elif self.motor.getOutputCurrentA() > self.stallCurrentLimitA \
                        and self.currentUpdateTimeS - self.lastMoveTimeS > self.stallTimeLimitS:
                    result = True
        else:
            self.lastMoveTimeS = self.currentUpdateTimeS


        self.lastDirectionSign = self.currentMotorDirectionSign
        self.lastMotorPosRad = self.currentMotorPosRad
        return result



    def update(self):
        if self.motor.getControlState() != MotorControlStates.POSITION:
            self._clearMonitorState()
        else:
            self.currentUpdateTimeS = Timer.getFPGATimestamp()
            if self.lastMoveTimeS is None:
                self.lastMoveTimeS = self.currentUpdateTimeS

            self._stalled = self._isMotorStalled()






