from utils.robotIdentification import RobotTypes
class RobotDependentConstants:
    def __init__(self):
        self.robotConstants = {
            RobotTypes.Spires2023: {
                "DT_FL_WHEEL_CANID": 2,
                "DT_FL_AZMTH_CANID": 3,
                "DT_FR_WHEEL_CANID": 4,
                "DT_FR_AZMTH_CANID": 5,
                "DT_BL_WHEEL_CANID": 6,
                "DT_BL_AZMTH_CANID": 7,
                "DT_BR_AZMTH_CANID": 8,
                "DT_BR_WHEEL_CANID": 9,
                "SWERVE_WHEEL_GEAR_RATIO": 5.50,   # Base Low
                #"SWERVE_WHEEL_GEAR_RATIO": 5.08,  # Base Medium
                #"SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "WIDTH": 16.5,
                "LENGTH": 26.5,
                "FL_OFFSET_DEG": 169.2+90+180,
                "FR_OFFSET_DEG": -49.7,
                "BL_OFFSET_DEG": -56.2+180,
                "BR_OFFSET_DEG": -11.2-90+180,
                "GYRO": "NAVX", # "NAVX", # "ADIS16470_IMU",
            },
            RobotTypes.Spires2024: {
                "DT_FL_WHEEL_CANID": 2,
                "DT_FL_AZMTH_CANID": 3,
                "DT_FR_WHEEL_CANID": 4,
                "DT_FR_AZMTH_CANID": 5,
                "DT_BL_WHEEL_CANID": 6,
                "DT_BL_AZMTH_CANID": 7,
                "DT_BR_AZMTH_CANID": 8,
                "DT_BR_WHEEL_CANID": 9,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71, # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "WIDTH": 22.5,
                "LENGTH": 26.5,
                "FL_OFFSET_DEG": 177.4-90,
                "FR_OFFSET_DEG": 0.7,
                "BL_OFFSET_DEG": 125.4-180,
                "BR_OFFSET_DEG": 117.5-90-180,
                "GYRO": "ADIS16470_IMU",
            },
        }

    def get(self):
        return self.robotConstants
