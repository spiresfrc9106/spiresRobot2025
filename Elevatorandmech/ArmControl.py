# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Elevator

# It is definitely buggy and untested, but it gives us a great framework on how to control an elevator.


from playingwithfusion import TimeOfFlight
from Elevatorandmech.ArmConstants import MAX_ARM_ACCEL_DPS2, MAX_ARM_VEL_DPS, ARM_GEARBOX_GEAR_RATIO, ARM_SPOOL_RADIUS_M
from utils.calibration import Calibration
from utils.units import sign
from utils.signalLogging import log
#from utils.constants import ARM_LM_CANID, ARM_RM_CANID, ARM_TOF_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from rev import SparkLowLevel
from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer

ARM_MOTOR_CANID = 24

class ArmControl(metaclass=Singleton):
    def __init__(self):

        # Arm Motors
        self.Rmotor = WrapperedSparkMax(ARM_MOTOR_CANID, "ArmMotor", brakeMode=True)



        # FF and proportional gain constants
        self.kV = Calibration(name="Arm kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Arm kS", default=0.1, units="V")
        self.kG = Calibration(name="Arm kG", default=0.25, units="V")
        self.kP = Calibration(name="Arm kP", default=0.05, units="V/rad error")

        # Set P gain on motor
        self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)

        # Profiler
        self.maxV = Calibration(name="Arm Max Vel", default=MAX_ARM_VEL_DPS, units="mps")
        self.maxA = Calibration(name="Arm Max Accel", default=MAX_ARM_ACCEL_DPS2, units="mps2")
        self.profiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxV.get(),self.maxA.get()))

        self.actualPosDeg = 0
        self.stopped = True

        # Limit switch code; bottom for resetting offset and ensuring it starts correctly, top for saftey to stop from spinning
    

        # Absolute Sensor mount offsets
        # After mounting the sensor, these should be tweaked one time
        # in order to adjust whatever the sensor reads into the reference frame
        # of the mechanism
        self.absOffsetM = 0.074

        # Relative Encoder Offsets
        # Releative encoders always start at 0 at power-on
        # However, we may or may not have the mechanism at the "zero" position when we powered on
        # These variables store an offset which is calculated from the absolute sensors
        # to make sure the relative sensors inside the encoders accurately reflect
        # the actual position of the mechanism
        self.relEncOffsetM = 0.0
        # Create a motion profile with the given maximum velocity and maximum
        # acceleration constraints for the next setpoint.

        self.armAngleGoalDeg = 0

        self.profiledPos = 0.0
        self.curUnprofiledPosCmd = 0.0

        # See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
        self.desState = TrapezoidProfile.State(self.armAngleGoalDeg,0)



    def _motorRadArmAngleDeg(self, motorRad):
        return 0 # TODO Put a place holder in here
    
    def _armAngleDegToMotorRad(self, armAngleDeg):
        return 0 # TODO Put a place holder in here
    
    def _ArmAngleVeltoMotorVel(self, armAngleVelDPS):
        return 0# TODO Put a place holder in here
    
    def getArmAngleDeg(self):
        return self.actualPosDeg
    
    #return the height of the Arm as measured by the absolute sensor in meters


    def update(self):
        self.actualPosDeg = 0 # TOOD replace this with a read of the absolute externalAbsoluteEncoder.

        # Update motor closed-loop calibration
        if(self.kP.isChanged()):
            self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)

        if self.stopped:
            self.Rmotor.setVoltage(0.0)
            self.profiledPos = self.actualPosDeg
        else:
            curState = self.profiler.State()
            self.profiler.calculate(0.02, curState, self.desState)

            self.profiledPos = curState.position

            motorPosCmd = self._heightToMotorRad(curState.position)
            motorVelCmd = self._heightVeltoMotorVel(curState.velocity)

            vFF = self.kV.get() * motorVelCmd  + self.kS.get() * sign(motorVelCmd) \
                + self.kG.get()

            self.motor.setPosCmd(motorPosCmd, vFF)


    def setAngleGoalDeg(self, goalDeg):
        pass
