# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Elevator

# It is definitely buggy and untested, but it gives us a great framework on how to control an elevator.


from Elevatorandmech.ElevatorandMechConstants import ELEV_HEIGHT, MAX_ELEV_ACCEL_MPS2, MAX_ELEV_VEL_MPS, \
    ELEV_GEARBOX_GEAR_RATIO, ELEV_SPOOL_RADIUS_M
from utils.calibration import Calibration
from utils.units import sign
from utils.signalLogging import log
# from utils.constants import ELEV_LM_CANID, ELEV_RM_CANID, ELEV_TOF_CANID
from utils.singleton import Singleton
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer

TEST_MOTOR_CANID = 20

class MotorControl(metaclass=Singleton):
    def __init__(self):

        # Elevator Motors
        self.Rmotor = WrapperedSparkMax(TEST_MOTOR_CANID, "Test_Motor", brakeMode=True)

        # Set P gain on motor
        self.Rmotor.setPID(0.05, 0.0, 0.0)



    def update(self, desiredSpeed):
        if desiredSpeed!=0:
            #rpm -> rps
            #rpm to rad per minute *2*3.14/60
            self.Rmotor.setVelCmd(desiredSpeed *2*3.14/60,1)
        else:
            self.Rmotor.setVoltage(0.0)
        self.Rmotor.getMotorVelocityRadPerSec()
        self.Rmotor.getAppliedOutput()



