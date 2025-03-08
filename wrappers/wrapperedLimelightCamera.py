import math
import random
from typing import Tuple
import wpilib
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Twist2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from drivetrain.drivetrainPhysical import (
    kinematics,
    ROBOT_TO_LEFT_CAM,
    ROBOT_TO_RIGHT_CAM,
    ROBOT_TO_FRONT_CAM,
)
from drivetrain.poseEstimation.drivetrainPoseTelemetry import DrivetrainPoseTelemetry
from utils.faults import Fault
from utils.signalLogging import addLog
from wrappers.wrapperedPoseEstPhotonCamera import WrapperedPoseEstPhotonCamera

from wpimath.geometry import Pose3d, Rotation3d, Translation3d
from sensors.limelight import Limelight

from dataclasses import dataclass
import wpilib
from wpimath.units import feetToMeters, degreesToRadians
from wpimath.geometry import Pose2d, Rotation2d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonCamera import setVersionCheckEnabled
from utils.fieldTagLayout import FieldTagLayout
from utils.faults import Fault
from ntcore import NetworkTableInstance


@dataclass
class LimelightCameraPoseObservation:
    time: float
    estFieldPose: Pose2d
    xyStdDev: float = 1.0  # std dev of error in measurment, units of meters.
    rotStdDev: float = degreesToRadians(50.0)  # std dev of measurement, in units of radians


class WrapperedPoseEstLimelight:
    def __init__(self, camName, robotToCam):
        setVersionCheckEnabled(False)

        self.cam = Limelight(robotToCam, camName)

        self.disconFault = Fault(f"LL Camera {camName} not sending data")
        self.timeoutSec = 1.0
        self.poseEstimates: list[LimelightCameraPoseObservation] = []
        self.robotToCam = robotToCam

        self.CamPublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic("/positionbyLL" + camName, Pose2d)
            .publish()
        )

        self.MetaTag2CamPublisher = (
            NetworkTableInstance.getDefault()
            .getStructTopic("/TESTINGposbyLL" + camName, Pose2d)
            .publish()
        )

        self.targetLength = 0
        addLog("test_targets_seen_limelight", lambda: self.targetLength, "")

    def update(self, prevEstPose:Pose2d):
        self.cam.update()

        self.poseEstimates = []

        if not self.cam.isConnected():
            # Faulted - no estimates, just return.
            self.disconFault.setFaulted()
            return

        #res = self.cam.getLatestResult()
        # broken in photonvision 2.4.2. Hack with the non-broken latency calcualtion
        # TODO: in 2025, fix this to actually use the real latency
        latency = 0.05  # a total guess
        obsTime = wpilib.Timer.getFPGATimestamp() - latency

        # Update our disconnected fault since we have something from the camera
        self.disconFault.setNoFault()

        if self.cam.april_tag_exists():
            #metatag2 is horrible, angles don't seem to work, sometimes jumps across field. using default.
            bestCandidate = self._toPose2d(botpose=self.cam.botpose)
            self.poseEstimates.append(LimelightCameraPoseObservation(obsTime, bestCandidate))
            self.CamPublisher.set(bestCandidate)
        self.targetLength = self.cam.get_april_length()


    def _toPose2d(self, botpose:list): #init: +9.0, +4.5
        #CAUSES WILD OSCILATION BETWEEN 180 and -180 or wtv:
        # return Pose3d(Translation3d(botpose[0], botpose[1], botpose[2]),Rotation3d(botpose[3], botpose[4], math.radians(botpose[5])),).toPose2d()

        return Pose2d(botpose[0], botpose[1], Rotation2d(math.radians(botpose[5])))


    def getPoseEstimates(self):
        return self.poseEstimates

