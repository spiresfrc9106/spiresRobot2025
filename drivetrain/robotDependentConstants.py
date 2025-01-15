from wpimath.system.plant import DCMotor
from utils.robotIdentification import RobotTypes
class RobotDependentConstants:
    def __init__(self):
        self.robotConstants = {
            RobotTypes.Spires2023: {
                "SWERVE_WHEEL_GEAR_RATIO": 5.50,   # Base Low
                #"SWERVE_WHEEL_GEAR_RATIO": 5.08,  # Base Medium
                #"SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RPS": DCMotor.NEO(1).freeSpeed,
                "WIDTH": 16.5,
                "LENGTH": 26.5,
                "MASS_LBS": 60,
                "FL_OFFSET_DEG": 169.2+90+180,
                "FR_OFFSET_DEG": -49.7,
                "BL_OFFSET_DEG": -56.2+180,
                "BR_OFFSET_DEG": -11.2-90+180,
                "GYRO": "NAVX", # "NAVX", # "ADIS16470_IMU",
            },
            RobotTypes.Spires2024: {
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71, # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RPS": DCMotor.neoVortex(1).freeSpeed,
                "WIDTH": 22.5,
                "LENGTH": 26.5,
                "MASS_LBS": 60,
                "FL_OFFSET_DEG": 177.4-90,
                "FR_OFFSET_DEG": 0.7,
                "BL_OFFSET_DEG": 125.4-180,
                "BR_OFFSET_DEG": 117.5-90-180,
                "GYRO": "ADIS16470_IMU",
            },
            RobotTypes.SpiresTestBoard: {
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RPS": DCMotor.neoVortex(1).freeSpeed,
                "WIDTH": 22.5,
                "LENGTH": 26.5,
                "MASS_LBS": 60,
                "FL_OFFSET_DEG": 0,
                "FR_OFFSET_DEG": 0,
                "BL_OFFSET_DEG": 0,
                "BR_OFFSET_DEG": 0,
                "GYRO": "NoGyro",
            },
            RobotTypes.SpiresRoboRioV1: {
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RPS": DCMotor.neoVortex(1).freeSpeed,
                "WIDTH": 22.5,
                "LENGTH": 26.5,
                "MASS_LBS": 60,
                "FL_OFFSET_DEG": 0,
                "FR_OFFSET_DEG": 0,
                "BL_OFFSET_DEG": 0,
                "BR_OFFSET_DEG": 0,
                "GYRO": "NoGyro",
            },
        }

    def get(self):
        return self.robotConstants
