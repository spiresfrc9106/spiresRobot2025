import wpilib
from ntcore import NetworkTableInstance
from wpimath.geometry._geometry import Rotation2d
from wpimath.units import inchesToMeters
from wpimath.system.plant import DCMotor
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d, Rotation2d
import wpilib

from utils.robotIdentification import RobotIdentification, RobotTypes
from wrappers.wrapperedPoseEstPhotonCamera import WrapperedPoseEstPhotonCamera
from wrappers.wrapperedLimelightCamera import wrapperedLimilightCameraFactory
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedSparkFlex import WrapperedSparkFlex

# Camera Mount Offsets
# These are relative to the robot origin
# which is in the center of the chassis on the ground
ROBOT_TO_LEFT_CAM = Transform3d(
    Translation3d(
        inchesToMeters(-10), inchesToMeters(-12), inchesToMeters(22)  # X  # Y  # Z
    ),
    Rotation3d.fromDegrees(0, -10, 90.0),  # Roll  # Pitch  # Yaw
)

ROBOT_TO_RIGHT_CAM = Transform3d(
    Translation3d(
        inchesToMeters(-10), inchesToMeters(12), inchesToMeters(22)  # X  # Y  # Z
    ),
    Rotation3d.fromDegrees(0, -10, -90.0),  # Roll  # Pitch  # Yaw
)

ROBOT_TO_FRONT_CAM = Transform3d(
    Translation3d(
        #inchesToMeters(-7.5), inchesToMeters(+11), inchesToMeters(13)  # X  # Y  # Z
        inchesToMeters(-7.5), inchesToMeters(11), inchesToMeters(13)  # X  # Y  # Z
),
    Rotation3d.fromDegrees(0.0, -10, 0.0),  # Roll  # Pitch  # Yaw
)

ROBOT_TO_LIME_1 = Transform3d(
    Translation3d(
        inchesToMeters(0), inchesToMeters(0), inchesToMeters(0)  # X:0.3556  # Y:(-0.25)-0.00635  # Z:0.2032
    ),
    Rotation3d.fromDegrees(0.0, 0.0, 0.0),  # Roll  # Pitch  # Yaw
)

# if wpilib.RobotBase.isSimulation() == False:
#     ROBOT_TO_LIME_1 = Pose3d(Translation3d(0,0,0), Rotation3d(Rotation2d(0)))

COMMON_CAMS = [
    {
        "CAM": WrapperedPoseEstPhotonCamera("RIGHT_CAM", ROBOT_TO_RIGHT_CAM),
        "POSE_EST_LOG_NAME": "photonR",
        "PUBLISHER":
            (
                NetworkTableInstance.getDefault()
                .getStructTopic("/RightCamPose", Pose3d)
                .publish()
            ),
        "ROBOT_TO_CAM": ROBOT_TO_RIGHT_CAM,
        "WEIGH_IN_FILTER": False,
    },
    {
        "CAM": WrapperedPoseEstPhotonCamera("LEFT_CAM", ROBOT_TO_LEFT_CAM),
        "POSE_EST_LOG_NAME": "photonL",
        "PUBLISHER":
            (
                NetworkTableInstance.getDefault()
                .getStructTopic("/LeftCamPose", Pose3d)
                .publish()
            ),
        "ROBOT_TO_CAM": ROBOT_TO_LEFT_CAM,
        "WEIGH_IN_FILTER": False,
    },
    {
        "CAM": WrapperedPoseEstPhotonCamera("FRONT_CAM", ROBOT_TO_FRONT_CAM),
        "POSE_EST_LOG_NAME": "photonF",
        "PUBLISHER":
            (
                NetworkTableInstance.getDefault()
                .getStructTopic("/FrontCamPose", Pose3d)
                .publish()
            ),
        "ROBOT_TO_CAM": ROBOT_TO_FRONT_CAM,
        "WEIGH_IN_FILTER": False,
    },
    {
        "CAM": wrapperedLimilightCameraFactory("limelight-br", ROBOT_TO_LIME_1),
        "POSE_EST_LOG_NAME": "limeli-br",
        "PUBLISHER":
            (
                NetworkTableInstance.getDefault()
                .getStructTopic("/Limili-brPose", Pose3d)
                .publish()
            ),
        "ROBOT_TO_CAM": ROBOT_TO_LIME_1,
        "WEIGH_IN_FILTER": True,
    },
    {
        "CAM": wrapperedLimilightCameraFactory("limelight-fl", ROBOT_TO_LIME_1),
        "POSE_EST_LOG_NAME": "limeli-fl",
        "PUBLISHER":
            (
                NetworkTableInstance.getDefault()
                .getStructTopic("/Limili-flPose", Pose3d)
                .publish()
            ),
        "ROBOT_TO_CAM": ROBOT_TO_LIME_1,
        "WEIGH_IN_FILTER": True,
    },
    {
        "CAM": wrapperedLimilightCameraFactory("limelight-fr", ROBOT_TO_LIME_1),
        "POSE_EST_LOG_NAME": "limeli-fr",
        "PUBLISHER":
            (
                NetworkTableInstance.getDefault()
                .getStructTopic("/Limili-frPose", Pose3d)
                .publish()
            ),
        "ROBOT_TO_CAM": ROBOT_TO_LIME_1,
        "WEIGH_IN_FILTER": True,
    },
]

class DrivetrainDependentConstants:
    def __init__(self):
        self.drivetrainConstants = {
            RobotTypes.Spires2023: {
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMax,
                "SWERVE_WHEEL_GEAR_RATIO": 5.50,   # Base Low
                #"SWERVE_WHEEL_GEAR_RATIO": 5.08,  # Base Medium
                #"SWERVE_WHEEL_GEAR_RATIO": 4.71,  # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RPS": DCMotor.NEO(1).freeSpeed,
                "WIDTH": 16.5,
                "LENGTH": 26.5,
                "MASS_LBS": 32, #changed because coach jeremy lifted the robot and felt it was that
                "FL_OFFSET_DEG": 169.2+90+180,
                "FR_OFFSET_DEG": -49.7,
                "BL_OFFSET_DEG": -56.2+180,
                "BR_OFFSET_DEG": -11.2-90+180,
                "GYRO": "NAVX", # "NAVX", # "ADIS16470_IMU",
                "CAMS": [],
                "HAS_DRIVETRAIN": True,
                "USE_PHOTON_NAV": False,
            },
            RobotTypes.Spires2025: {
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkFlex,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71, # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RPS": DCMotor.neoVortex(1).freeSpeed,
                "WIDTH": 26.0,
                "LENGTH": 33.0,
                "MASS_LBS": 104,
                "FL_OFFSET_DEG": 177.4-90+180-4,
                "FR_OFFSET_DEG": 0.7-123,
                "BL_OFFSET_DEG": 125.4-180-2,
                "BR_OFFSET_DEG": 117.5-90-180-155,
                "GYRO": "ADIS16470_IMU",
                "CAMS": COMMON_CAMS,
                "HAS_DRIVETRAIN": True,
                "USE_PHOTON_NAV": True,
            },
            RobotTypes.Spires2025Sim: {
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkFlex,
                # "SWERVE_WHEEL_GEAR_RATIO": 5.50, # Base Low
                # "SWERVE_WHEEL_GEAR_RATIO": 5.08, # Base Medium
                "SWERVE_WHEEL_GEAR_RATIO": 4.71, # Base High
                "SWERVE_WHEEL_DIAMETER_IN": 3.0,
                "SWERVE_WHEEL_MAX_SPEED_RPS": DCMotor.neoVortex(1).freeSpeed,
                "WIDTH": 22.5,
                "LENGTH": 26.5,
                "MASS_LBS": 60,
                "FL_OFFSET_DEG": 0,
                "FR_OFFSET_DEG": 0,
                "BL_OFFSET_DEG": 0,
                "BR_OFFSET_DEG": 0,
                "CAMS": COMMON_CAMS,
                "GYRO": "ADIS16470_IMU",
                "HAS_DRIVETRAIN": True,
                "USE_PHOTON_NAV": True,
            },
            RobotTypes.SpiresTestBoard: {
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMax,
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
                "CAMS": [],
                "HAS_DRIVETRAIN": False,
                "USE_PHOTON_NAV": False,
            },
            RobotTypes.SpiresRoboRioV1: {
                "WHEEL_MOTOR_WRAPPER": WrapperedSparkMax,
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
                "CAMS": [],
                "HAS_DRIVETRAIN": False,
                "USE_PHOTON_NAV": False,
            },
        }

    def get(self):
        return self.drivetrainConstants[RobotIdentification().getRobotType()]

drivetrainDepConstants = DrivetrainDependentConstants().get()
