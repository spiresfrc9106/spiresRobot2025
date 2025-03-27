import sys
import gc
import wpilib
import ntcore as nt
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from dashboard import Dashboard
from Elevatorandmech.ElevatorControl import ElevatorControl, elevDepConstants
from Elevatorandmech.NewArmControl import ArmControl, armDepConstants
from Elevatorandmech.RobotPoserCommon import PoseDirectorCommon
from Elevatorandmech.RobotPoserDriver import PoseDirectorDriver
from Elevatorandmech.RobotPoserOperator import PoseDirectorOperator
from testingMotors.motorCtrl import MotorControl, motorDepConstants
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.controlStrategies.trajectoryGuts import TrajectoryGuts
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainControl import DrivetrainControl
from drivetrain.DrivetrainDependentConstants import drivetrainDepConstants
from humanInterface.driverInterface import DriverInterface
from humanInterface.operatorInterface import OperatorInterface
from humanInterface.ledControl import LEDControl
from navigation.forceGenerators import PointObstacle
from ultrasound.ultrasound import Ultrasound
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.signalLogging import logUpdate, getNowLogger
from utils.calibration import CalibrationWrangler
from utils.faults import FaultWrangler
from utils.crashLogger import CrashLogger
from utils.rioMonitor import RIOMonitor
from utils.robotIdentification import RobotIdentification
from utils.singleton import destroyAllSingletonInstances
from utils.powerMonitor import PowerMonitor
from utils.allianceTransformUtils import onRed
from utils.units import deg2Rad

from webserver.webserver import Webserver
from AutoSequencerV2.autoSequencer import AutoSequencer

class MyRobot(wpilib.TimedRobot):

    #########################################################
    ## Common init/update for all modes
    def robotInit(self):
        print("robotInit has run")
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        remoteRIODebugSupport()

        print(f"robot type = {RobotIdentification().getRobotType()} serialNumber={RobotIdentification().serialNumber}")

        self.crashLogger = CrashLogger()
        wpilib.LiveWindow.disableAllTelemetry()
        self.webserver = Webserver()

        self.driveTrain = None
        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            print(f"drivetrainDepConstants['HAS_DRIVETRAIN']={drivetrainDepConstants['HAS_DRIVETRAIN']}")
            self.driveTrain = DrivetrainControl()
            self.tcTraj = self.driveTrain.tcTraj

        self.arm = None
        if armDepConstants['HAS_ARM']:
            self.arm = ArmControl()

        self.ultrasound = Ultrasound()

        self.elev = None
        if elevDepConstants['HAS_ELEVATOR']:
            self.elev= ElevatorControl()

        self.poseDirectorCommon = PoseDirectorCommon
        self.poseDirectorCommon.initialize(self.dInt, self.oInt, self.driveTrain, self.arm, self.elev)
        self.poseDirectorDriver = PoseDirectorDriver()
        self.poseDirectorDriver.initialize()
        self.poseDirectorOperator = PoseDirectorOperator()
        self.poseDirectorOperator.initialize()


        self.autodrive = AutoDrive()

        self.stt = SegmentTimeTracker(longLoopPrintEnable=False, epochTracerEnable=False)

        self.dInt = DriverInterface()
        self.oInt = OperatorInterface()
        self.ledCtrl = LEDControl()

        self.autoSequencer = AutoSequencer()

        self.dashboard = Dashboard()

        #self.rioMonitor = RIOMonitor()
        if False:
            self.pwrMon = PowerMonitor()
        else:
            self.pwrMon = None

        if motorDepConstants['HAS_MOTOR_TEST']:
            self.motorCtrlFun = MotorControl()

        # Normal robot code updates every 20ms, but not everything needs to be that fast.
        # Register slower-update periodic functions
        if self.pwrMon is not None:
            self.addPeriodic(self.pwrMon.update, 0.2, 0.0)
        self.addPeriodic(self.crashLogger.update, 1.0, 0.0)
        self.addPeriodic(CalibrationWrangler().update, 0.5, 0.0)
        self.addPeriodic(FaultWrangler().update, 0.2, 0.0)

        self.autoHasRun = False

        self.logger1 = getNowLogger('now1', 'sec')
        self.logger2 = getNowLogger('now2', 'sec')
        self.logger3 = getNowLogger('now3', 'sec')

        gc.freeze()
        self.count=0


    def robotPeriodic(self):
        self.logger1.logNow(nt._now())

        self.stt.start()

        if self.count == 10:
            gc.freeze()
        self.dInt.update()
        self.stt.mark("Driver Interface")
        self.oInt.update()
        self.stt.mark("Operator Interface")

        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            self.driveTrain.update()
            self.stt.mark("Drivetrain")

        self.autodrive.updateTelemetry()
        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            self.driveTrain.poseEst._telemetry.setCurAutoDriveWaypoints(self.autodrive.getWaypoints())
            self.driveTrain.poseEst._telemetry.setCurObstacles(self.autodrive.rfp.getObstacleStrengths())
        self.stt.mark("Telemetry")
        self.logger2.logNow(nt._now())

        self.ultrasound.update()
        self.stt.mark("Ultrasound")

        self.ledCtrl.setAutoDrive(self.autodrive.isRunning())
        self.ledCtrl.setStuck(self.autodrive.rfp.isStuck())
        self.ledCtrl.update()
        self.stt.mark("LED Ctrl")

        logUpdate()
        self.count += 1
        self.stt.end()
        self.logger3.logNow(nt._now())

        if self.autoSequencer.getMenuChange():
            self.dashboard.resetWidgets()
            self.dashboard = Dashboard()
            self.autoSequencer.acknowledgeDashboardReset()

    #########################################################
    ## Autonomous-Specific init and update
    def autonomousInit(self):
        print("autonomousInit has run")

        # Start up the autonomous sequencer
        self.autoSequencer.initialize()

        #consider resetting gyro here

        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            # Use the autonomous routines starting pose to init the pose estimator
            self.driveTrain.poseEst.setKnownPose(self.autoSequencer.getStartingPose())  #position set.
            self.driveTrain.tcPoseEst.setKnownPose(self.autoSequencer.getStartingPose())

        # Mark we at least started autonomous
        self.autoHasRun = True # pylint: disable=attribute-defined-outside-init

        if armDepConstants['HAS_ARM']:
            self.arm.forceReInit()
            self.arm.initialize()

        if elevDepConstants['HAS_ELEVATOR']:
            self.elev.forceReInit()
            self.elev.initialize()

    def autonomousPeriodic(self):

        self.autoSequencer.update()
        self.poseDirectorDriver.update(isAuton=True)
        self.poseDirectorOperator.update(isAuton=True)

        # Operators cannot control in autonomous
        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            self.driveTrain.setManualCmd(DrivetrainCommand())

        if armDepConstants['HAS_ARM']:
            self.arm.update()
            self.stt.mark("Arm-auto")

        if elevDepConstants['HAS_ELEVATOR']:
            self.elev.update()
            self.stt.mark("Elevator-auto")

    def autonomousExit(self):
        self.autoSequencer.end()


    #########################################################
    ## Teleop-Specific init and update
    def teleopInit(self):
        print("teleopInit has run")
        # clear existing telemetry trajectory
        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            self.driveTrain.poseEst._telemetry.setCurAutoTrajectory(None)
            self.driveTrain.tcPoseEst._telemetry.setCurAutoTrajectory(None)

        # If we're starting teleop but haven't run auto, set a nominal default pose
        # This is needed because initial pose is usually set by the autonomous routine
        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            # xyzzy todo Noah, can we change this so that:
            # we always have a default autonoumous pose?
            # that if auto hasn't run, we set our default poss to the default, or selected autonoumous pose?
            # -Thanks Coach Mike
            if not self.autoHasRun:
                if onRed():
                    self.driveTrain.poseEst.setKnownPose(
                        Pose2d(10.4279, 3.722, Rotation2d(0))
                    )
                else:
                    self.driveTrain.poseEst.setKnownPose(
                        Pose2d(7.1411, 3.722, Rotation2d(deg2Rad(180)))
                    )

        if armDepConstants['HAS_ARM']:
            self.arm.initialize()

        if elevDepConstants['HAS_ELEVATOR']:
            self.elev.initialize()

        # Default to No trajectory in Teleop, The PoseDirector does send commands through in teleop
        Trajectory().setCmdFromChoreoAuton(None)
        self.tcTraj.setCmdFromChoreoAuton(None)

    def teleopPeriodic(self):
        # TODO - this is technically one loop delayed, which could induce lag
        # Probably not noticeable, but should be corrected.

        if drivetrainDepConstants['HAS_DRIVETRAIN']:
            self.driveTrain.setManualCmd(self.dInt.getCmd(), self.dInt.getRobotRelative())

        self.poseDirectorDriver.update()
        self.poseDirectorOperator.update()

        if self.dInt.getGyroResetCmd():
            if drivetrainDepConstants['HAS_DRIVETRAIN']:
                self.driveTrain.resetGyro()


        if self.dInt.getCreateObstacle():
            if drivetrainDepConstants['HAS_DRIVETRAIN']:
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

        if armDepConstants['HAS_ARM']:
            #self.arm.setPosVelocityGoal(posGoalDeg=self.oInt.getDesArmAngleDeg(), velocityGoalDegps=0.0)
            self.arm.update()
            self.stt.mark("Arm-teleop")


        if motorDepConstants['HAS_MOTOR_TEST']:
            self.motorCtrlFun.update(self.dInt.getMotorTestPowerRpm())


        if elevDepConstants['HAS_ELEVATOR']:
            self.elev.update()
            self.stt.mark("Elevator-teleop")

    #########################################################
    ## Disabled-Specific init and update
    def disabledPeriodic(self):
        self.autoSequencer.updateMode()
        Trajectory().trajHDC.updateCals()
        self.tcTraj.trajHDC.updateCals()


    def disabledInit(self):
        self.poseDirectorOperator.setDashboardState(1) # State 1, put the autonomous menu back up on the webserver dashboard
        self.autoSequencer.updateMode(True)
        if armDepConstants['HAS_ARM'] and self.arm is not None:
            self.arm.disable()

        if elevDepConstants['HAS_ELEVATOR'] and self.elev is not None:
            self.elev.disable()

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

