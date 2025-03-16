import math
from wpimath.units import inchesToMeters
from wpimath.system.plant import DCMotor
from wpimath.geometry import Translation2d, Transform3d, Translation3d, Rotation3d
from wpimath.kinematics import SwerveDrive4Kinematics

from drivetrain.DrivetrainDependentConstants import drivetrainDepConstants
from utils.units import lbsToKg
from utils.units import deg2Rad
from utils.units import in2m
from utils.robotIdentification import RobotIdentification, RobotTypes
from wrappers.wrapperedRevThroughBoreEncoder import WrapperedRevThroughBoreEncoder
from wrappers.wrapperedLimelightCamera import wrapperedLimilightCameraFactory
from wrappers.wrapperedPoseEstPhotonCamera import WrapperedPoseEstPhotonCamera
from sensors.limelight import Limelight

"""
Defines the physical dimensions and characteristics of the drivetrain
"""

###################################################################
# Physical dimensions and mass distribution

# Wheel base half width: Distance from the center of the frame rail
# out to the center of the "contact patch" where the wheel meets the ground
WHEEL_BASE_HALF_WIDTH_M = inchesToMeters(drivetrainDepConstants["WIDTH"] / 2.0)
WHEEL_BASE_HALF_LENGTH_M = inchesToMeters(drivetrainDepConstants["LENGTH"] / 2.0)

# Additional distance from the wheel contact patch out to the edge of the bumper
BUMPER_THICKNESS_M = inchesToMeters(2.5)

# Total mass includes robot, battery, and bumpers
# more than the "weigh-in" weight
if RobotIdentification().isSpiresRobot():
    ROBOT_MASS_KG = lbsToKg(drivetrainDepConstants['MASS_LBS'])
else:
    ROBOT_MASS_KG = lbsToKg(60)

# Model the robot's moment of intertia as a square slab
# slightly bigger than wheelbase with axis through center
ROBOT_MOI_KGM2 = 1.0 / 12.0 * ROBOT_MASS_KG * WHEEL_BASE_HALF_WIDTH_M * WHEEL_BASE_HALF_LENGTH_M * math.pow(2.2,
                                                                                                            2) * 2

# SDS MK4i Swerve Module Ratios
# See https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=39598777172081
WHEEL_GEAR_RATIO_L1 = 8.41
WHEEL_GEAR_RATIO_L2 = 6.75
WHEEL_GEAR_RATIO_L3 = 6.12 #TODO what is WHEEL_GEAR_RATIO_L3
AZMTH_GEAR_RATIO = 12.8 # TODO fix me up

## CHANGE THIS DEPENDING ON WHICH MODULE GEAR RATIO IS INSTALLED
if RobotIdentification().getRobotType() == RobotTypes.Main:
    WHEEL_GEAR_RATIO = WHEEL_GEAR_RATIO_L3
elif RobotIdentification().getRobotType() == RobotTypes.Practice:
    WHEEL_GEAR_RATIO = WHEEL_GEAR_RATIO_L2
elif RobotIdentification().isSpiresRobot():
    WHEEL_GEAR_RATIO = drivetrainDepConstants['SWERVE_WHEEL_GEAR_RATIO']
else:
    WHEEL_GEAR_RATIO = WHEEL_GEAR_RATIO_L3

# carpet/roughtop interface fudge factor
# This accounts for the fact that roughtop tread
# sinks into the carpet slightly. Determined empirically
# by driving the robot a known distance, seeing the measured distance in software,
# and adjusting this factor till the measured distance matches known
# Might have to be different for colson wheels?
WHEEL_FUDGE_FACTOR = 0.9238

# Nominal 4-inch diameter swerve drive wheels
# https:#www.swervedrivespecialties.com/collections/mk4i-parts/products/billet-wheel-4d-x-1-5w-bearing-bore
WHEEL_RADIUS_IN = drivetrainDepConstants["SWERVE_WHEEL_DIAMETER_IN"] / 2.0 * WHEEL_FUDGE_FACTOR


# Utility conversion functions to go between drivetrain "linear" measurements and wheel motor rotational measurements
def dtLinearToMotorRot(lin):
    # lin - meters per second at wheel contact patch
    # return - radians per second of motor shaft
    return lin / (inchesToMeters(WHEEL_RADIUS_IN)) * WHEEL_GEAR_RATIO


def dtMotorRotToLinear(rot):
    # rot - radians per second of motor shaft
    # return = meters per second at wheel contact patch
    return rot * (inchesToMeters(WHEEL_RADIUS_IN)) / WHEEL_GEAR_RATIO


# Drivetrain Performance Mechanical limits
# Nominal calculations (ideal)
if RobotIdentification().isSpiresRobot():
    MAX_DT_MOTOR_SPEED_RPS = drivetrainDepConstants['SWERVE_WHEEL_MAX_SPEED_RPS']
else:
    MAX_DT_MOTOR_SPEED_RPS = DCMotor.NEO(1).freeSpeed
    #MAX_DT_MOTOR_SPEED_RPS = DCMotor.neoVortex(1).freeSpeed
MAX_DT_LINEAR_SPEED_MPS = MAX_DT_MOTOR_SPEED_RPS / WHEEL_GEAR_RATIO * in2m(WHEEL_RADIUS_IN)
# Fudged max expected performance

speed_multiple_at_comp = 2

MAX_FWD_REV_SPEED_MPS = MAX_DT_LINEAR_SPEED_MPS * 0.98 * speed_multiple_at_comp  # fudge factor due to gearbox losses
MAX_STRAFE_SPEED_MPS = MAX_DT_LINEAR_SPEED_MPS * 0.98 * speed_multiple_at_comp  # fudge factor due to gearbox losses
MAX_ROTATE_SPEED_RAD_PER_SEC = deg2Rad(
    360.0
) * speed_multiple_at_comp  # Fixed at the maximum rotational speed we'd want.
# Accelerations - also a total guess
MAX_TRANSLATE_ACCEL_MPS2 = (
    #MAX_FWD_REV_SPEED_MPS / 0.50
    MAX_FWD_REV_SPEED_MPS/ 0.10
)  # 0-full time of 0.5 second - this is a guestimate xyzzy - investigate making this smaller
MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = (
    MAX_ROTATE_SPEED_RAD_PER_SEC / 0.25
)  # 0-full time of 0.25 second - this is a guestaimate


# Mechanical mounting offsets of the encoder & magnet within the shaft
# Must be updated whenever the module is reassembled
# Procedure:
# 0 - Put the robot up on blocks.
# 1 - Reset all these values to 0, deploy code
# 2 - Pull up dashboard with encoder readings (in radians)
# 3 - Using a square, twist the modules by hand until they are aligned with the robot's chassis
# 4 - Read out the encoder readings for each module, put them here
# 5 - Redeploy code, verify that the  encoder readings are correct as each module is manually rotated


if RobotIdentification().isSpiresRobot():
    # Perhaps we invert the swerve module azimuth motor
    INVERT_AZMTH_MOTOR = False
    INVERT_AZMTH_ENCODER = True

    # Perhaps we invert the swerve module wheel motor drive direction
    FL_INVERT_WHEEL_MOTOR = False
    FR_INVERT_WHEEL_MOTOR = False
    BL_INVERT_WHEEL_MOTOR = False
    BR_INVERT_WHEEL_MOTOR = False
else:
    # Perhaps we invert the swerve module azimuth motor
    INVERT_AZMTH_MOTOR = True
    INVERT_AZMTH_ENCODER = False

    # Perhaps we invert the swerve module wheel motor drive direction
    FL_INVERT_WHEEL_MOTOR = True
    FR_INVERT_WHEEL_MOTOR = True
    BL_INVERT_WHEEL_MOTOR = False
    BR_INVERT_WHEEL_MOTOR = False

# todo they moved the configuration closer to the code.

if RobotIdentification().getRobotType() == RobotTypes.Main:
    FR_ENCODER_MOUNT_OFFSET_RAD = 0.8412
    FL_ENCODER_MOUNT_OFFSET_RAD = 0.2412
    BR_ENCODER_MOUNT_OFFSET_RAD = 1.259
    BL_ENCODER_MOUNT_OFFSET_RAD = 1.777
elif RobotIdentification().isSpiresRobot():
    FL_ENCODER_MOUNT_OFFSET_RAD = deg2Rad(drivetrainDepConstants["FL_OFFSET_DEG"])
    FR_ENCODER_MOUNT_OFFSET_RAD = deg2Rad(drivetrainDepConstants["FR_OFFSET_DEG"])
    BL_ENCODER_MOUNT_OFFSET_RAD = deg2Rad(drivetrainDepConstants["BL_OFFSET_DEG"])
    BR_ENCODER_MOUNT_OFFSET_RAD = deg2Rad(drivetrainDepConstants["BR_OFFSET_DEG"])
else:
    FR_ENCODER_MOUNT_OFFSET_RAD = 0.8412
    FL_ENCODER_MOUNT_OFFSET_RAD = 0.2412
    BR_ENCODER_MOUNT_OFFSET_RAD = 1.259
    BL_ENCODER_MOUNT_OFFSET_RAD = 1.777

# Module Indices (for ease of array manipulation)
FL = 0
FR = 1
BL = 2
BR = 3

# Function make a swerve module azimuth encoder reader object
def wrapperedSwerveDriveAzmthEncoder(azmthEncoderPortIdx, moduleName, azmthOffsetRad, inverted):
    return WrapperedRevThroughBoreEncoder(
        port=azmthEncoderPortIdx,
        name=moduleName,
        mountOffsetRad=azmthOffsetRad,
        dirInverted=inverted
    )

WHEEL_MOTOR_WRAPPER = drivetrainDepConstants["WHEEL_MOTOR_WRAPPER"]


CAMS = drivetrainDepConstants["CAMS"]

# Array of translations from robot's origin (center bottom, on floor) to the module's contact patch with the ground
robotToModuleTranslations = []
robotToModuleTranslations.append(
    Translation2d(WHEEL_BASE_HALF_LENGTH_M, WHEEL_BASE_HALF_WIDTH_M)
)
robotToModuleTranslations.append(
    Translation2d(WHEEL_BASE_HALF_LENGTH_M, -WHEEL_BASE_HALF_WIDTH_M)
)
robotToModuleTranslations.append(
    Translation2d(-WHEEL_BASE_HALF_LENGTH_M, WHEEL_BASE_HALF_WIDTH_M)
)
robotToModuleTranslations.append(
    Translation2d(-WHEEL_BASE_HALF_LENGTH_M, -WHEEL_BASE_HALF_WIDTH_M)
)

# WPILib Kinematics object
kinematics = SwerveDrive4Kinematics(
    robotToModuleTranslations[FL],
    robotToModuleTranslations[FR],
    robotToModuleTranslations[BL],
    robotToModuleTranslations[BR],
)
