import sys
import gc
import wpilib
import ntcore as nt
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from dashboard import Dashboard
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainControl import DrivetrainControl
from humanInterface.driverInterface import DriverInterface
from humanInterface.ledControl import LEDControl
from navigation.forceGenerators import PointObstacle
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.signalLogging import logUpdate, getNowLogger
from utils.calibration import CalibrationWrangler
from utils.faults import FaultWrangler
from utils.crashLogger import CrashLogger
from utils.rioMonitor import RIOMonitor
from utils.singleton import destroyAllSingletonInstances
from utils.powerMonitor import PowerMonitor
from webserver.webserver import Webserver
from AutoSequencerV2.autoSequencer import AutoSequencer
from Elevatorandmech.armtest import ArmControl
from Elevatorandmech.elevatortest import ElevatorControl
from wpimath.geometry import Pose2d
from poser.poser import Poser

class MyRobot(wpilib.TimedRobot):

    #########################################################
    ## Common init/update for all modes
    def robotInit(self):
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        remoteRIODebugSupport()

        self.crashLogger = CrashLogger()
        wpilib.LiveWindow.disableAllTelemetry()
        self.webserver = Webserver()

        self.driveTrain = DrivetrainControl()
        self.autodrive = AutoDrive()

        self.stt = SegmentTimeTracker()      

        self.dInt = DriverInterface()
        self.ledCtrl = LEDControl()

        self.autoSequencer = AutoSequencer()

        self.dashboard = Dashboard()

        self.rioMonitor = RIOMonitor()
        self.pwrMon = PowerMonitor()

        # Normal robot code updates every 20ms, but not everything needs to be that fast.
        # Register slower-update periodic functions
        self.addPeriodic(self.pwrMon.update, 0.2, 0.0)
        self.addPeriodic(self.crashLogger.update, 1.0, 0.0)
        self.addPeriodic(CalibrationWrangler().update, 0.5, 0.0)
        self.addPeriodic(FaultWrangler().update, 0.2, 0.0)

        self.autoHasRun = False

        self.logger1 = getNowLogger('now1', 'sec')
        self.logger2 = getNowLogger('now2', 'sec')
        self.logger3 = getNowLogger('now3', 'sec')

        # test ones, not real!
        self.arm = ArmControl()
        self.elevator = ElevatorControl()
        self.poser = Poser()

        gc.freeze()
        self.count=0


    def robotPeriodic(self):
        self.logger1.logNow(nt._now())

        self.stt.start()

        if self.count == 10:
            gc.freeze()
        self.dInt.update()
        self.stt.mark("Driver Interface")

        self.driveTrain.update()
        self.stt.mark("Drivetrain")

        self.autodrive.updateTelemetry()
        self.driveTrain.poseEst._telemetry.setCurAutoDriveWaypoints(self.autodrive.getWaypoints())
        self.driveTrain.poseEst._telemetry.setCurObstacles(self.autodrive.rfp.getObstacleStrengths())
        self.stt.mark("Telemetry")

        self.logger2.logNow(nt._now())

        self.ledCtrl.setAutoDrive(self.autodrive.isRunning())
        self.ledCtrl.setStuck(self.autodrive.rfp.isStuck())
        self.ledCtrl.update()
        self.stt.mark("LED Ctrl")

        logUpdate()
        self.count += 1
        self.stt.end()
        self.logger3.logNow(nt._now())

    #########################################################
    ## Autonomous-Specific init and update
    def autonomousInit(self):

        # Start up the autonomous sequencer
        self.autoSequencer.initialize()

        # Use the autonomous rouines starting pose to init the pose estimator
        self.driveTrain.poseEst.setKnownPose(self.autoSequencer.getStartingPose())

        # Mark we at least started autonomous
        self.autoHasRun = True # pylint: disable=attribute-defined-outside-init

    def autonomousPeriodic(self):

        self.autoSequencer.update()

        # Operators cannot control in autonomous
        self.driveTrain.setManualCmd(DrivetrainCommand())

    def autonomousExit(self):
        self.autoSequencer.end()

    #########################################################
    ## Teleop-Specific init and update
    def teleopInit(self):
        # clear existing telemetry trajectory
        self.driveTrain.poseEst._telemetry.setCurAutoTrajectory(None)

        # If we're starting teleop but haven't run auto, set a nominal default pose
        # This is needed because initial pose is usually set by the autonomous routine
        if not self.autoHasRun:
            self.driveTrain.poseEst.setKnownPose(
                Pose2d(1.0, 1.0, Rotation2d(0.0))
            )


    def teleopPeriodic(self):
        # TODO - this is technically one loop delayed, which could induce lag
        # Probably not noticeable, but should be corrected.
        self.driveTrain.setManualCmd(self.dInt.getCmd())
        self.poser.update()

        if self.dInt.getGyroResetCmd():
            self.driveTrain.resetGyro()


        if self.dInt.getCreateObstacle():
            # For test purposes, inject a series of obstacles around the current pose
            ct = self.driveTrain.poseEst.getCurEstPose().translation()
            tfs = [
                #Translation2d(1.7, -0.5),
                #Translation2d(0.75, -0.75),
                #Translation2d(1.7, 0.5),
                Translation2d(0.75, 0.75),
                Translation2d(2.0, 0.0),
                Translation2d(0.0, 1.0),
                Translation2d(0.0, -1.0),
            ]
            for tf in tfs:
                obs = PointObstacle(location=(ct+tf), strength=0.5)
                self.autodrive.rfp.addObstacleObservation(obs)

        self.autodrive.setRequest(self.dInt.getNavToSpeaker(), self.dInt.getNavToPickup())

        # No trajectory in Teleop
        Trajectory().setCmd(None)

    #########################################################
    ## Disabled-Specific init and update
    def disabledPeriodic(self):
        self.autoSequencer.updateMode()
        Trajectory().trajHDC.updateCals()

    def disabledInit(self):
        self.autoSequencer.updateMode(True)

    #########################################################
    ## Test-Specific init and update
    def testInit(self):
        wpilib.LiveWindow.setEnabled(False)

    def testPeriodic(self):
        pass

    #########################################################
    ## Cleanup
    def endCompetition(self):

        # Sometimes `robopy test pyfrc_test.py` will invoke endCompetition() without completing robotInit(),
        # this will create a confusing exception here because we can reach self.rioMonitor.stopThreads()
        # when self.rioMonitor does not exist.
        # To prevent the exception and confusion, we only call self.rioMonitor.stopThreads() when exists.
        rioMonitorExists = getattr(self, "rioMonitor", None)
        if rioMonitorExists is not None:
            self.rioMonitor.stopThreads()

        destroyAllSingletonInstances()
        super().endCompetition()

def remoteRIODebugSupport():
    if __debug__ and "run" in sys.argv:
        print("Starting Remote Debug Support....")
        try:
            import debugpy  # pylint: disable=import-outside-toplevel
        except ModuleNotFoundError:
            pass
        else:
            debugpy.listen(("0.0.0.0", 5678))
            debugpy.wait_for_client()