# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Elevator

# It is definitely buggy and untested, but it gives us a great framework on how to control an elevator.


from playingwithfusion import TimeOfFlight
from Elevatorandmech.ElevatorandMechConstants import ELEV_HEIGHT, MAX_ELEV_ACCEL_MPS2, MAX_ELEV_VEL_MPS, ELEV_GEARBOX_GEAR_RATIO, ELEV_SPOOL_RADIUS_M
from utils.calibration import Calibration
from utils.units import sign
from utils.signalLogging import log
#from utils.constants import ELEV_LM_CANID, ELEV_RM_CANID, ELEV_TOF_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from rev import SparkLowLevel
from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer

ELEV_LM_CANID = 23
ELEV_RM_CANID = 21
ELEV_TOF_CANID = 22

class ElevatorControl(metaclass=Singleton):
    def __init__(self):

        # Elevator Motors
        self.Rmotor = WrapperedSparkMax(ELEV_RM_CANID, "ElevatorMotorRight", brakeMode=True)
        self.LMotor = WrapperedSparkMax(ELEV_LM_CANID, "ElevatorMotorLeft", brakeMode=True)
        #we don't know if we want to invert LMotor (left) or not when we follow RMotor (right), automatically assumed False
        self.LMotor.setFollow(ELEV_RM_CANID)

        # Coral Scoring Heights in meters
        self.L1_Height = Calibration(name="Height of L1", units="m", default=0.5842 - ELEV_HEIGHT)
        self.L2_Height = Calibration(name="Height of L2", units="m", default=0.9398 - ELEV_HEIGHT)
        self.L3_Height = Calibration(name="Height of L3", units="m", default=1.397 - ELEV_HEIGHT)
        self.L4_Height = Calibration(name="Height of L4", units="m", default=2.159 - ELEV_HEIGHT)

        # FF and proportional gain constants
        self.kV = Calibration(name="Elevator kV", default=0.02, units="V/rps")
        self.kS = Calibration(name="Elevator kS", default=0.1, units="V")
        self.kG = Calibration(name="Elevator kG", default=0.25, units="V")
        self.kP = Calibration(name="Elevator kP", default=0.05, units="V/rad error")

        # Set P gain on motor
        self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)

        # Profiler
        self.maxV = Calibration(name="Elevator Max Vel", default=MAX_ELEV_VEL_MPS, units="mps")
        self.maxA = Calibration(name="Elevator Max Accel", default=MAX_ELEV_ACCEL_MPS2, units="mps2")
        self.profiler = TrapezoidProfile(TrapezoidProfile.Constraints(self.maxV.get(),self.maxA.get()))
        #go to wpilib online documentation to learn more about the trapezoid (very cool)

        self.actualPos = 0
        self.stopped = True

        # Try to set a small current limit and decide when we're on the bottom using this, and turn off the motor when it doesn't need to spin anymore.  then up the current limit as needed


        # Absolute Sensor mount offsets: use laser range finders to measure distance? either sensor or revolution count

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

        self.heightGoal = self.L1_Height.get()

        self.profiledPos = 0.0
        self.curUnprofiledPosCmd = 0.0

        # See: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html
        self.desState = TrapezoidProfile.State(self.heightGoal,0)



    def _RmotorRadToHeight(self, RmotorRad):
        return RmotorRad * 1/ELEV_GEARBOX_GEAR_RATIO * (ELEV_SPOOL_RADIUS_M) - self.relEncOffsetM
    
    def _heightToMotorRad(self, elevLin):
        return ((elevLin + self.relEncOffsetM)*1/(ELEV_SPOOL_RADIUS_M) * ELEV_GEARBOX_GEAR_RATIO)
    
    def _heightVeltoMotorVel(self, elevLinVel):
        return (elevLinVel *1/(ELEV_SPOOL_RADIUS_M) * ELEV_GEARBOX_GEAR_RATIO)
    
    def getHeightM(self):
        return self._RmotorRadToHeight(self.Rmotor.getMotorPositionRad()) 
    
    #return the height of the elevator as measured by the absolute sensor in meters
    def _getAbsHeight(self):
        return self.heightAbsSen.getRange() / 1000.0 - self.absOffsetM

    # This routine uses the absolute sensors to adjust the offsets for the relative sensors
    # so that the relative sensors match reality.
    # It should be called.... infrequently. Likely once shortly after robot init.
    def initFromAbsoluteSensor(self):
        # Reset offsets to zero, so the relative sensor get functions return
        # just whatever offset the relative sensor currently has.
        self.relEncOffsetM = 0.0

        # New Offset = real height - what height says?? 
        self.relEncOffsetM = self._getAbsHeight() - self.getHeightM()

    def update(self):
        self.actualPos = self.getHeightM()

        # Update motor closed-loop calibration
        if(self.kP.isChanged()):
            self.Rmotor.setPID(self.kP.get(), 0.0, 0.0)

        if(self.stopped):
            self.Rmotor.setVoltage(0.0)
            self.profiledPos = self.actualPos
        else:
            curState = self.profiler.State()
            self.profiler.calculate(0.02, curState, self.desState)

            self.profiledPos = curState.position

            motorPosCmd = self._heightToMotorRad(curState.position)
            motorVelCmd = self._heightVeltoMotorVel(curState.velocity)
            # set our feed forward to 0 at the start so we're not throwing extra voltage into the motor, then see if their feed forward calc makes sense
            vFF = self.kV.get() * motorVelCmd  + self.kS.get() * sign(motorVelCmd) \
                + self.kG.get()

            self.Rmotor.setPosCmd(motorPosCmd, vFF)
            self.LMotor.setFollow(self.Rmotor.ctrl, True)

    # we're gonna make this height goal where we just set the height off the ground that we want the elevator pivot joint to be at
    def setHeightGoal(self, height_in):
        self.heightGoal = height_in
        self.desState = TrapezoidProfile.State(self.heightGoal,0)

