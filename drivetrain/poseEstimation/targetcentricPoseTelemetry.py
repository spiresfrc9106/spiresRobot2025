import math

import wpilib
from wpimath.trajectory import Trajectory
from wpimath.geometry import Pose2d, Pose3d, Transform2d, Rotation2d

from utils.allianceTransformUtils import transform
from drivetrain.drivetrainPhysical import CAMS
from drivetrain.drivetrainPhysical import  robotToModuleTranslations
from wrappers.wrapperedPoseEstPhotonCamera import CameraPoseObservation
from utils.signalLogging import addLog
from ntcore import NetworkTableInstance


class TargetCentricPoseTelemetry:
    """
    Helper class to wrapper sending all drivetrain Pose related information
    to dashboards
    """

    def __init__(self):
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("TC Pose 2D", self.field)
        self.curTraj = Trajectory()
        self.curTrajWaypoints = []
        self.fixedObstacles = []
        self.fullObstacles = []
        self.thirdObstacles = []
        self.almostGoneObstacles = []

        self.desPose = Pose2d()

        self.camPublishers = []
        self.robotToCams = []
        self.variabchnag =0
        self.theInterestingValue = []
        self.interestingTracker = []

        icount = 0
        for camConfig in CAMS:
            self.camPublishers.append(camConfig['PUBLISHER'])
            self.robotToCams.append(camConfig['ROBOT_TO_CAM'])
            camName = camConfig['POSE_EST_LOG_NAME']
            icount += 1

        self.camPublishersAndRobotToCams = zip(self.camPublishers, self.robotToCams)

        self.visionPoses = []
        self.modulePoses = []

    def setDesiredPose(self, desPose):
        self.desPose = desPose

    def setCurAutoDriveWaypoints(self, waypoints:list[Pose2d]):
        self.curTrajWaypoints = waypoints

    def addVisionObservations(self, observations:list[CameraPoseObservation]):
        if(len(observations) > 0):
            for obs in observations:
                self.visionPoses.append(obs.estFieldPose)

    def setCurObstacles(self, obstacles):
        self.fixedObstacles, self.fullObstacles, self.thirdObstacles, self.almostGoneObstacles = obstacles

    def clearVisionObservations(self):
        self.visionPoses = []

    def update(self, estPose:Pose2d, moduleAngles):
        self.field.getRobotObject().setPose(estPose)
        self.field.getObject("ModulePoses").setPoses(
            [
                estPose.transformBy(Transform2d(robotToModuleTranslations[0], moduleAngles[0])),
                estPose.transformBy(Transform2d(robotToModuleTranslations[1], moduleAngles[1])),
                estPose.transformBy(Transform2d(robotToModuleTranslations[2], moduleAngles[2])),
                estPose.transformBy(Transform2d(robotToModuleTranslations[3], moduleAngles[3])),
            ]
        )

        self.field.getObject("desPose").setPose(self.desPose)
        self.field.getObject("desTraj").setTrajectory(self.curTraj)
        self.field.getObject("desTrajWaypoints").setPoses(self.curTrajWaypoints)
        self.field.getObject("curObstaclesFixed").setPoses([Pose2d(x, Rotation2d()) for x in self.fixedObstacles])
        self.field.getObject("curObstaclesFull").setPoses([Pose2d(x, Rotation2d()) for x in self.fullObstacles])
        self.field.getObject("curObstaclesThird").setPoses([Pose2d(x, Rotation2d()) for x in self.thirdObstacles])
        self.field.getObject("curObstaclesAlmostGone").setPoses(
            [Pose2d(x, Rotation2d()) for x in self.almostGoneObstacles]
        )

        self.field.getObject("visionObservations").setPoses(self.visionPoses)
        self.visionPoses = []

        self.theInterestingValue = []
        icount = 0
        for publisher, robotToCam in self.camPublishersAndRobotToCams:
            publisher.set(Pose3d(estPose).transformBy(robotToCam))
            icount += 1

    def setCurAutoTrajectory(self, trajIn):
        """Display a specific trajectory on the robot Field2d

        Args:
            trajIn (WPI Trajectory): The trajectory to display
        """
        if(trajIn is not None):
            self.curTraj = trajIn
        else:
            self.curTraj = Trajectory()

    def setChoreoTrajectory(self, trajIn):
        """Display a specific trajectory on the robot Field2d

        Args:
            trajIn (Choreo Trajectory object): The trajectory to display
        """
        # Transform choreo state list into useful trajectory for telemetry
        if trajIn is not None:
            stateList = []

            # For visual appearance and avoiding sending too much over NT,
            # make sure we only send a sampled subset of the positions
            sampTime = 0
            while sampTime < trajIn.getTotalTime():
                stateList.append(
                    self._choreoToWPIState(transform(trajIn.sample(sampTime)))
                )
                sampTime += 0.5

            # Make sure final pose is in the list
            stateList.append(self._choreoToWPIState(transform(trajIn.samples[-1])))

            self.curTraj = Trajectory(stateList)
        else:
            self.curTraj = Trajectory()

    # PathPlanner has a built in "to-wpilib" representation, but it doesn't
    # account for holonomic heading. Fix that.
    def _choreoToWPIState(self, inVal):
        return Trajectory.State(
            acceleration=0,
            pose=inVal.getPose(),
            t=inVal.timestamp,
            velocity=math.hypot(inVal.velocityX, inVal.velocityY),
        )
