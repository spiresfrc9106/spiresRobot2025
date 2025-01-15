from wpilib import ADIS16470_IMU
from wpimath.geometry import Rotation2d
import navx
from drivetrain.robotDependentConstants import RobotDependentConstants

from utils.robotIdentification import RobotIdentification

class WrapperedNoGyro():
    def __init__(self):
        pass

    def getGyroAngleRotation2d(self):
        return Rotation2d(0.0)

    def isConnected(self):
        return False

class WrapperedNavx(navx.AHRS):
    """
    Class to wrap a navx
    """
    def __init__(self):
        super().__init__(comType=navx.AHRS.NavXComType.kMXP_SPI, updateRate=50)

    def getGyroAngleRotation2d(self)->Rotation2d:
        return self.getRotation2d()

class WrapperedAdis16470Imu(ADIS16470_IMU):

    def getGyroAngleRotation2d(self)->Rotation2d:
        return Rotation2d().fromDegrees(self.getAngle(self.getYawAxis()))

def wrapperedGyro():
    result = None
    if RobotIdentification().isSpiresRobot():
        robotDepConstants = RobotDependentConstants().get()[RobotIdentification().getRobotType()]
        print(f'GYRO is {robotDepConstants["GYRO"]}')
        if robotDepConstants["GYRO"] == "NAVX":
            result = WrapperedNavx()
        elif robotDepConstants["GYRO"] == "ADIS16470_IMU":
            result = WrapperedAdis16470Imu()
        else:
            result = WrapperedNoGyro()
    else:
        result = WrapperedAdis16470Imu()
    return result
