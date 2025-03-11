from ntcore import NetworkTableInstance
from wpimath.units import inchesToMeters
from wpimath.system.plant import DCMotor
from wpimath.geometry import Pose3d, Transform3d, Translation3d, Rotation3d

from utils.robotIdentification import RobotIdentification, RobotTypes
from wrappers.wrapperedPoseEstPhotonCamera import WrapperedPoseEstPhotonCamera
from wrappers.wrapperedLimelightCamera import wrapperedLimilightCameraFactory

# Camera Mount Offsets
# These are relative to the robot origin
# which is in the center of the chassis on the ground
ROBOT_TO_LEFT_CAM = Transform3d(
    Translation3d(
        inchesToMeters(3.7), inchesToMeters(13.8), inchesToMeters(7.4)  # X  # Y  # Z
    ),
    Rotation3d.fromDegrees(0, -30, 90.0),  # Roll  # Pitch  # Yaw
)

ROBOT_TO_RIGHT_CAM = Transform3d(
    Translation3d(
        inchesToMeters(3.7), inchesToMeters(-13.8), inchesToMeters(7.4)  # X  # Y  # Z
    ),
    Rotation3d.fromDegrees(0, -30, -90.0),  # Roll  # Pitch  # Yaw
)

ROBOT_TO_FRONT_CAM = Transform3d(
    Translation3d(
        inchesToMeters(14), inchesToMeters(2.75), inchesToMeters(9)  # X  # Y  # Z
    ),
    Rotation3d.fromDegrees(0.0, 3.0, 0.0),  # Roll  # Pitch  # Yaw
)

ROBOT_TO_LIME_1 = Transform3d(
    Translation3d(
        inchesToMeters(14), inchesToMeters(-8), inchesToMeters(8)  # X:0.3556  # Y:(-0.25)-0.00635  # Z:0.2032
    ),
    Rotation3d.fromDegrees(0.0, 11.0, 0.0),  # Roll  # Pitch  # Yaw
)

COMMON_CAMS = [
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
    },
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
    },
    {
        "CAM": wrapperedLimilightCameraFactory("limelight", ROBOT_TO_LIME_1),
        "POSE_EST_LOG_NAME": "imeli1",
        "PUBLISHER":
            (
                NetworkTableInstance.getDefault()
                .getStructTopic("/Limili1Pose", Pose3d)
                .publish()
            ),
        "ROBOT_TO_CAM": ROBOT_TO_LIME_1,
    },
]

class DrivetrainDependentConstants:
    def __init__(self):
        self.drivetrainConstants = {
            RobotTypes.Spires2023: {
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
                "CAMS": [
                    #WrapperedPoseEstPhotonCamera("LEFT_CAM", ROBOT_TO_LEFT_CAM),
                    #WrapperedPoseEstPhotonCamera("RIGHT_CAM", ROBOT_TO_RIGHT_CAM),
                    #WrapperedPoseEstPhotonCamera("FRONT_CAM", ROBOT_TO_FRONT_CAM),
                    #wrapperedLimilightCameraFactory("limelight", ROBOT_TO_LIME_1), #limelight-three
                ],
                # don't include "_"
                "POSE_EST_LOG_NAMES": [
                    #"photonL",
                    #"photonR",
                    #"photonF",
                    #"limeli1",
                ],
                "HAS_DRIVETRAIN": True,
            },
            RobotTypes.Spires2025: {
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
                "CAMS": [
                    #WrapperedPoseEstPhotonCamera("LEFT_CAM", ROBOT_TO_LEFT_CAM),
                    #WrapperedPoseEstPhotonCamera("RIGHT_CAM", ROBOT_TO_RIGHT_CAM),
                    #WrapperedPoseEstPhotonCamera("FRONT_CAM", ROBOT_TO_FRONT_CAM),
                    #wrapperedLimilightCameraFactory("limelight", ROBOT_TO_LIME_1), #limelight-three
                ],
                # don't include "_"
                "POSE_EST_LOG_NAMES": [
                    #"photonL",
                    #"photonR",
                    #"photonF",
                    #"limeli1",
                ],
                "HAS_DRIVETRAIN": True,
            },
            RobotTypes.Spires2025Sim: {
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
                "CAMS": COMMON_CAMS,
                "GYRO": "ADIS16470_IMU",
                "HAS_DRIVETRAIN": True,
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
                "CAMS": [
                    #WrapperedPoseEstPhotonCamera("LEFT_CAM", ROBOT_TO_LEFT_CAM),
                    #WrapperedPoseEstPhotonCamera("RIGHT_CAM", ROBOT_TO_RIGHT_CAM),
                    #WrapperedPoseEstPhotonCamera("FRONT_CAM", ROBOT_TO_FRONT_CAM),
                    #wrapperedLimilightCameraFactory("limelight", ROBOT_TO_LIME_1), #limelight-three
                ],
                # don't include "_"
                "POSE_EST_LOG_NAMES": [
                    #"photonL",
                    #"photonR",
                    #"photonF",
                    #"limeli1",
                ],
                "HAS_DRIVETRAIN": False,
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
                "CAMS": [
                    #WrapperedPoseEstPhotonCamera("LEFT_CAM", ROBOT_TO_LEFT_CAM),
                    #WrapperedPoseEstPhotonCamera("RIGHT_CAM", ROBOT_TO_RIGHT_CAM),
                    #WrapperedPoseEstPhotonCamera("FRONT_CAM", ROBOT_TO_FRONT_CAM),
                    #wrapperedLimilightCameraFactory("limelight", ROBOT_TO_LIME_1), #limelight-three
                ],
                # don't include "_"
                "POSE_EST_LOG_NAMES": [
                    #"photonL",
                    #"photonR",
                    #"photonF",
                    #"limeli1",
                ],
                "HAS_DRIVETRAIN": False,
            },
        }

    def get(self):
        return self.drivetrainConstants[RobotIdentification().getRobotType()]
