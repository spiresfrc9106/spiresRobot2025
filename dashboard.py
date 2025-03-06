import wpilib
from AutoSequencerV2.autoSequencer import AutoSequencer
from dashboardWidgets.autoChooser import AutoChooser
from dashboardWidgets.swerveState import SwerveState
from dashboardWidgets.icon import Icon
from dashboardWidgets.text import Text
from dashboardWidgets.fieldPose import FieldPose
from dashboardWidgets.lineGauge import LineGauge
from dashboardWidgets.progressBar import ProgressBar
from utils.faults import FaultWrangler
from utils.signalLogging import addLog
from webserver.webserver import Webserver


class Dashboard:
    def __init__(self):
        webServer = Webserver()

        # THIS MUST GO ON TOP!
        webServer.addDashboardWidget(Text(0,0,"/SmartDashboard/robot_current_state"))


        webServer.addDashboardWidget(Icon(45, 45, "/SmartDashboard/isRedIconState", "#FF0000", "allianceRed"))
        webServer.addDashboardWidget(Icon(55, 45, "/SmartDashboard/isBlueIconState", "#0000FF", "allianceBlue"))
        webServer.addDashboardWidget(Icon(65, 45, "/SmartDashboard/PE Vision Targets Seen", "#00FF00", "vision"))

        webServer.addDashboardWidget(Text(50, 75, "/SmartDashboard/faultDescription"))
        webServer.addDashboardWidget(SwerveState(85, 15))
        webServer.addDashboardWidget(FieldPose(20, 40))
        webServer.addDashboardWidget(ProgressBar(20,60,"/SmartDashboard/ytest_position_final_x", 0, 20, 0, 20))

        webServer.addDashboardWidget(
            AutoChooser(
                50,
                10,
                AutoSequencer().getDelayModeNTTableName(),
                AutoSequencer().getDelayModeList(),
            )
        )
        webServer.addDashboardWidget(
            AutoChooser(
                50,
                20,
                AutoSequencer().getMainModeNTTableName(),
                AutoSequencer().getMainModeList(),
            )
        )

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