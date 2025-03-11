from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.geometry import Pose2d, Rotation2d
#from Autonomous.commands.driveForwardSlowCommand import DriveForwardSlowCommand
from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator
from drivetrain.swerveModuleControl import SwerveModuleControl
from drivetrain.swerveModuleGainSet import SwerveModuleGainSet
from drivetrain.drivetrainPhysical import (
    FL_ENCODER_MOUNT_OFFSET_RAD,
    MAX_FWD_REV_SPEED_MPS,
    FR_ENCODER_MOUNT_OFFSET_RAD,
    BL_ENCODER_MOUNT_OFFSET_RAD,
    BR_ENCODER_MOUNT_OFFSET_RAD,
    FL_INVERT_WHEEL_MOTOR,
    FR_INVERT_WHEEL_MOTOR,
    BL_INVERT_WHEEL_MOTOR,
    BR_INVERT_WHEEL_MOTOR,
    INVERT_AZMTH_MOTOR,
    INVERT_AZMTH_ENCODER,
    kinematics,
    WHEEL_MOTOR_WRAPPER,
)
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from utils.singleton import Singleton
from utils.allianceTransformUtils import onRed
from utils.constants import (DT_FL_WHEEL_CANID, 
                             DT_FL_AZMTH_CANID, 
                             DT_FR_WHEEL_CANID, 
                             DT_FR_AZMTH_CANID, 
                             DT_BL_WHEEL_CANID, 
                             DT_BL_AZMTH_CANID,
                             DT_BR_WHEEL_CANID,
                             DT_BR_AZMTH_CANID,
                             DT_FL_AZMTH_ENC_PORT,
                             DT_FR_AZMTH_ENC_PORT,
                             DT_BL_AZMTH_ENC_PORT,
                             DT_BR_AZMTH_ENC_PORT)
from wrappers.wrapperedGyro import wrapperedGyro
from Elevatorandmech.RobotPoser import PoseDirector

class DrivetrainControl(metaclass=Singleton):
    """
    Top-level control class for controlling a swerve drivetrain
    """

    def __init__(self):
        self.gyro = wrapperedGyro()
        self.modules = []
        self.modules.append(
            SwerveModuleControl("FL", WHEEL_MOTOR_WRAPPER, DT_FL_WHEEL_CANID, DT_FL_AZMTH_CANID, DT_FL_AZMTH_ENC_PORT,
                                FL_ENCODER_MOUNT_OFFSET_RAD,
                                FL_INVERT_WHEEL_MOTOR, INVERT_AZMTH_MOTOR, INVERT_AZMTH_ENCODER)
        )
        self.modules.append(
            SwerveModuleControl("FR", WHEEL_MOTOR_WRAPPER, DT_FR_WHEEL_CANID, DT_FR_AZMTH_CANID, DT_FR_AZMTH_ENC_PORT,
                                FR_ENCODER_MOUNT_OFFSET_RAD,
                                FR_INVERT_WHEEL_MOTOR, INVERT_AZMTH_MOTOR, INVERT_AZMTH_ENCODER)
        )
        self.modules.append(
            SwerveModuleControl("BL", WHEEL_MOTOR_WRAPPER, DT_BL_WHEEL_CANID, DT_BL_AZMTH_CANID, DT_BL_AZMTH_ENC_PORT,
                                BL_ENCODER_MOUNT_OFFSET_RAD,
                                BL_INVERT_WHEEL_MOTOR, INVERT_AZMTH_MOTOR, INVERT_AZMTH_ENCODER)
        )
        self.modules.append(
            SwerveModuleControl("BR", WHEEL_MOTOR_WRAPPER, DT_BR_WHEEL_CANID, DT_BR_AZMTH_CANID, DT_BR_AZMTH_ENC_PORT,
                                BR_ENCODER_MOUNT_OFFSET_RAD,
                                BR_INVERT_WHEEL_MOTOR, INVERT_AZMTH_MOTOR, INVERT_AZMTH_ENCODER)
        )

        self.coastCmd = False

        self.desChSpd = ChassisSpeeds()
        self.curDesPose = Pose2d()
        self.curManCmd = DrivetrainCommand()
        self.curCmd = DrivetrainCommand()

        self.gains = SwerveModuleGainSet()

        self.poseEst = DrivetrainPoseEstimator(self.getModulePositions(), self.gyro)

        self._updateAllCals()
        self.poser = PoseDirector()

    def setManualCmd(self, cmd: DrivetrainCommand):
        """Send commands to the robot for motion relative to the field

        Args:
            cmd (DrivetrainCommand): manual command input
        """
        self.curManCmd = cmd


    def setCoastCmd(self, coast:bool):
        self.coastCmd = coast

    def update(self):
        """
        Main periodic update, should be called every 20ms
        """
        curEstPose = self.poseEst.getCurEstPose()

        # Iterate through all strategies for controlling the drivetrain to
        # calculate the current drivetrain commands.

        self.curCmd = self.curManCmd
        self.curCmd = Trajectory().update(self.curCmd, curEstPose)
        self.curCmd = AutoDrive().update(self.curCmd, curEstPose)

        #yavin's interpret
        self.curCmd = self.poser.getDriveTrainCommand(self.curCmd)
        if hasattr(self.curCmd, "heading"):
            Trajectory().setCmd(self.curCmd)
            self.curCmd = Trajectory().update(self.curCmd, curEstPose)

        # Transform the current command to be robot relative
        tmp = ChassisSpeeds.fromFieldRelativeSpeeds(
            self.curCmd.velX, self.curCmd.velY, self.curCmd.velT, curEstPose.rotation()
        )
        self.desChSpd = _discretizeChSpd(tmp)

        # Set the desired pose for telemetry purposes
        self.poseEst._telemetry.setDesiredPose(self.curCmd.desPose)

        # pylint: disable=condition-evals-to-constant
        if (False and
            abs(self.desChSpd.vx) < 0.01 and
            abs(self.desChSpd.vy) < 0.01 and
            abs(self.desChSpd.omega) < 0.01 and
            not self.coastCmd):

            # When we're not moving, "toe in" the wheels to resist getting pushed around
            flModState = SwerveModuleState(angle=Rotation2d.fromDegrees(45), speed=0)
            frModState = SwerveModuleState(angle=Rotation2d.fromDegrees(-45), speed=0)
            blModState = SwerveModuleState(angle=Rotation2d.fromDegrees(45), speed=0)
            brModState = SwerveModuleState(angle=Rotation2d.fromDegrees(-45), speed=0)
            desModStates = (flModState, frModState, blModState, brModState)
        else:
            # Given the current desired chassis speeds, convert to module states
            desModStates = kinematics.toSwerveModuleStates(self.desChSpd)

        # Scale back commands if one corner of the robot is going too fast
        kinematics.desaturateWheelSpeeds(desModStates, MAX_FWD_REV_SPEED_MPS)

        # Send commands to modules and update
        for idx, module in enumerate(self.modules):
            module.setDesiredState(desModStates[idx])
            module.update()

        # Update the estimate of our pose
        self.poseEst.update(self.getModulePositions(), self.getModuleStates())

        # Update calibration values if they've changed
        if self.gains.hasChanged():
            self._updateAllCals()

    def _updateAllCals(self):
        # Helper function - updates all calibration on request
        for module in self.modules:
            module.setClosedLoopGains(self.gains)

    def getModulePositions(self):
        """
        Returns:
            Tuple of the actual module positions (as read from sensors)
        """
        return tuple(mod.getActualPosition() for mod in self.modules)
    
    def getModuleDesStates(self):
        """
        Returns:
            Tuple of the desired module states (as read from sensors)
        """
        return tuple(mod.getDesiredState() for mod in self.modules)

    def getModuleStates(self):
        """
        Returns:
            Tuple of the actual module speeds (as read from sensors)
        """
        return tuple(mod.getActualState() for mod in self.modules)

    def resetGyro(self):
        # Update pose estimator to think we're at the same translation,
        # but aligned facing downfield
        curTranslation = self.poseEst.getCurEstPose().translation()
        newGyroRotation = (
            Rotation2d.fromDegrees(180.0) if (onRed()) else Rotation2d.fromDegrees(0.0)
        )
        newPose = Pose2d(curTranslation, newGyroRotation)
        self.poseEst.setKnownPose(newPose)

    def getCurEstPose(self) -> Pose2d:
        # Return the current best-guess at our pose on the field.
        return self.poseEst.getCurEstPose()


def _discretizeChSpd(chSpd):
    """See https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/30
        Corrects for 2nd order kinematics
        Should be included in wpilib 2024, but putting here for now

    Args:
        chSpd (ChassisSpeeds): ChassisSpeeds input

    Returns:
        ChassisSpeeds: Adjusted ch speed
    """
    dt = 0.02
    poseVel = Pose2d(chSpd.vx * dt, chSpd.vy * dt, Rotation2d(chSpd.omega * dt))
    twistVel = Pose2d().log(poseVel)
    return ChassisSpeeds(twistVel.dx / dt, twistVel.dy / dt, twistVel.dtheta / dt)
