import wpilib
from AutoSequencerV2.autoSequencer import AutoSequencer
from dashboardWidgets.autoChooser import AutoChooser
from dashboardWidgets.swerveState import SwerveState
from dashboardWidgets.icon import Icon
from dashboardWidgets.text import Text
from dashboardWidgets.fieldPose import FieldPose
from dashboardWidgets.lineGauge import LineGauge
from dashboardWidgets.circularGauge import CircularGauge
from dashboardWidgets.progressBar import ProgressBar
from utils.faults import FaultWrangler
from utils.signalLogging import addLog
from webserver.webserver import Webserver
from dashboardWidgets.camera import Camera


class Dashboard:
    def __init__(self):
        webServer = Webserver()

        # state, errors, field, auton1, auton2, left, right, arm, elev, chain, scheme, progress, front l, front r, back

        # 0 state
        webServer.addDashboardWidget(Text(40, 40, "/SmartDashboard/RP/dashboardState"))

        # 1 errors
        webServer.addDashboardWidget(Text(50, 75, "/SmartDashboard/faultDescription"))

        # 2 field pose
        webServer.addDashboardWidget(FieldPose(20, 40))

        # 3 auton1
        webServer.addDashboardWidget(
            AutoChooser(
                50,
                10,
                AutoSequencer().getDelayModeNTTableName(),
                AutoSequencer().getDelayModeList(),
            )
        )
        # 4 auton2
        webServer.addDashboardWidget(
            AutoChooser(
                50,
                20,
                AutoSequencer().getMainModeNTTableName(),
                AutoSequencer().getMainModeList(),
            )
        )

        # 5 left
        webServer.addDashboardWidget(Icon(45, 45, "/SmartDashboard/isLeftReef", "#b942f5", "reefLeft"))
        # 6 right
        webServer.addDashboardWidget(Icon(55, 45, "/SmartDashboard/isRightReef", "#b942f5", "reefRight"))

        # 7 arm
        webServer.addDashboardWidget(CircularGauge(70, 80, "/SmartDashboard/RParm/pos", -90, 90, -100, 100))
        # 8 elev
        webServer.addDashboardWidget(CircularGauge(20, 80, "/SmartDashboard/RPelev/pos", 15, 60, 10, 65))
        # 9 chain
        webServer.addDashboardWidget(CircularGauge(20, 80, "/SmartDashboard/ytest_position_final_x", 0, 20, 0, 20))
        # 10 scheme progress
        webServer.addDashboardWidget(ProgressBar(20, 80, "/SmartDashboard/RP/schemeProg", 0, 100, 0, 100))

        # 11 cam1: front_l
        webServer.addDashboardWidget(Camera(10, 20, "http://limelight-fl.local:5800/"))
        # 12 cam2: front_r
        webServer.addDashboardWidget(Camera(10, 20, "http://limelight-fr.local:5800/"))
        # 13 cam3: back
        webServer.addDashboardWidget(Camera(10, 20, "http://limelight-br.local:5800/"))

        # extra stuff.
        # webServer.addDashboardWidget(Icon(45, 45, "/SmartDashboard/isRedIconState", "#FF0000", "allianceRed"))
        # webServer.addDashboardWidget(Icon(55, 45, "/SmartDashboard/isBlueIconState", "#0000FF", "allianceBlue"))
        # webServer.addDashboardWidget(Icon(65, 45, "/SmartDashboard/PE Vision Targets Seen", "#00FF00", "vision"))

        # Add logging for things that don't come from anywhere else


        addLog("isRedIconState",
               lambda: (
                   Icon.kON if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
                   else Icon.kOFF)
               )

        addLog("isBlueIconState",
               lambda: (
                   Icon.kON if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue
                   else Icon.kOFF)
               )

        addLog("faultIconState",
               lambda: (Icon.kBLINK_FAST if FaultWrangler().hasActiveFaults() else Icon.kOFF)
               )
