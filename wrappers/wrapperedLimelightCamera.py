from dataclasses import dataclass
import math

from ntcore import NetworkTableInstance
import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Transform3d, Pose3d, Translation3d

from utils.signalLogging import addLog
from wrappers.wrapperedPoseEstPhotonCamera import WrapperedPoseEstPhotonCamera

from sensors.limelight import Limelight

from photonlibpy.photonCamera import setVersionCheckEnabled
from utils.faults import Fault

@dataclass
class LimelightCameraPoseObservation:
    time: float
    estFieldPose: Pose2d
    xyStdDev: float  # std dev of error in measurment, units of meters.
    rotStdDev: float # std dev of measurement, in units of radians


class WrapperedPoseEstLimelight:
    def __init__(self, camName:str, robotToCam:Translation3d):
        setVersionCheckEnabled(False)

        print(f"WrapperedPoseEstLimelight camName {type(camName)} = {camName}")

        try:
            self.cam = Limelight(robotToCam, camName)
        except Exception as e:
            # Handle any exception
            print(f"An error occurred: {e}")
            self.cam = None

        self.disconFault = Fault(f"LL Camera {camName} not sending data")
        self.timeoutSec = 1.0
        self.poseEstimates: list[LimelightCameraPoseObservation] = []
        self.robotToCam: Translation3d = robotToCam

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

        self.xStdDev = 0
        self.yStdDev = 0
        self.tStdDev = 0

        addLog("ytest_limelight_sd_x", lambda: self.xStdDev, "")
        addLog("ytest_limelight_sd_y", lambda: self.yStdDev, "")
        addLog("ytest_limelight_sd_t", lambda: self.tStdDev, "")


        self.targetLength = 0
        addLog("ytest_targets_limelight_seen", lambda: self.targetLength, "")

    def update(self, prevEstPose:Pose2d):
        self.cam.update()

        self.poseEstimates = []

        if self.cam is None or not self.cam.isConnected():
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
            ta = self.cam.getTargetSize()
            self.xStdDev = self._getCameraStdDev(ta, measure_x=True)
            self.yStdDev = self._getCameraStdDev(ta, measure_y=True)
            self.tStdDev = self._getCameraStdDev(ta, measure_t=True)
            self.poseEstimates.append(LimelightCameraPoseObservation(obsTime, bestCandidate, max(self.xStdDev, self.yStdDev), self.tStdDev))
            self.CamPublisher.set(bestCandidate)
            secondCandidate = self._toPose2d(botpose=self.cam.botposemeta2)
            self.MetaTag2CamPublisher.set(secondCandidate)
        self.targetLength = self.cam.get_april_length()

    def _adjust(self, pos):
        return pos.transformBy(self.robotToCam.inverse()).toPose2d()

    def _toPose2d(self, botpose:list): #init: +9.0, +4.5
        #CAUSES WILD OSCILATION BETWEEN 180 and -180 or wtv:
        # return Pose3d(Translation3d(botpose[0], botpose[1], botpose[2]),Rotation3d(botpose[3], botpose[4], math.radians(botpose[5])),).toPose2d()

        return Pose2d(botpose[0], botpose[1], Rotation2d(math.radians(botpose[5]))) # initially: self._adjust(Pose3d())

    def getPoseEstFormatted(self):
        if self.cam is not None:
            return self._toPose2d(botpose=self.cam.botpose)
        else:
            return None

    def getPoseEstimates(self):
        return self.poseEstimates

    def _getCameraStdDev(self, ta, measure_x: bool = False, measure_y: bool = False, measure_t: bool = False):
        x = ta
        a = 0
        b = 1
        d = 0
        c = 1
        if measure_x:
            a = 0.370647
            b = 0.00096827
            d = -0.00299884
            c = 0.0492246
        if measure_y:
            a = 0.616315
            b = 0.00169949
            d = -0.0201106
            c = 0.174843
        if measure_t:
            a = 0.142239
            b = 0.00169949
            d = -0.00758282
            c = 0.0795458

        y = a * pow(b, x) + d * x + c
        return max(y, 0.0127)


def wrapperedLimilightCameraFactory(camName:str, robotToCam):
    if wpilib.RobotBase.isSimulation():
        print(f"In simulation substituting PhotonCamera for LimeLight Camera {camName}")
        wrapperedCam = WrapperedPoseEstPhotonCamera(camName, robotToCam)
    else:
        wrapperedCam = WrapperedPoseEstLimelight(camName, robotToCam)
    return wrapperedCam
