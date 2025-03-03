# Notes from Coach Mike, This code is from: https://github.com/RobotCasserole1736/RobotCasserole2025/tree/Traian_Elevator

# It is definitely buggy and untested, but it gives us a great framework on how to control an elevator.



from utils.signalLogging import log, getNowLogger
from utils.singleton import Singleton
from utils.robotIdentification import RobotTypes, RobotIdentification
from wrappers.wrapperedSparkMax import WrapperedSparkMax

class MotorDependentConstants:
    def __init__(self):
        self.motorDepConstants = {
            RobotTypes.Spires2023: {
                "HAS_MOTOR_TEST": False,
                "TEST_MOTOR_CANID": None,
            },
            RobotTypes.Spires2025: {
                "HAS_MOTOR_TEST": False,
                "TEST_MOTOR_CANID": None,
            },
            RobotTypes.SpiresTestBoard: {
                "HAS_MOTOR_TEST": False,
                "TEST_MOTOR_CANID": None,
        },
            RobotTypes.SpiresRoboRioV1: {
                "HAS_MOTOR_TEST": False,
                "TEST_MOTOR_CANID": None,
            },
        }

    def get(self):
        return self.motorDepConstants[RobotIdentification().getRobotType()]

motorDepConstants = MotorDependentConstants().get()


TEST_MOTOR_CANID = motorDepConstants['TEST_MOTOR_CANID']

class MotorControl(metaclass=Singleton):
    def __init__(self):

        # Elevator Motors
        self.Rmotor = WrapperedSparkMax(TEST_MOTOR_CANID, "Test_Motor", brakeMode=False, currentLimitA=20)

        # Set P gain on motor
        self.Rmotor.setPID(0.00005, 0.0, 0.0)
        self.motorRpmLogger = getNowLogger(f"Test_Motor_DesRpm", "rpm")



    def update(self, desiredSpeedRpm):
        if desiredSpeedRpm!=0:
            #rpm -> rps
            #rpm to rad per minute *2*3.14/60
            self.Rmotor.setVelCmd(desiredSpeedRpm *2*3.14/60,0)
        else:
            self.Rmotor.setVelCmd(0,0)
            self.Rmotor.setVoltage(0.0)
        self.motorRpmLogger.logNow(desiredSpeedRpm)
        self.Rmotor.getMotorVelocityRadPerSec()
        self.Rmotor.getAppliedOutput()



