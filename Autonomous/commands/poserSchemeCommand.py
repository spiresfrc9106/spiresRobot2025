from enum import IntEnum
from wpilib import Timer
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainControl import DrivetrainControl
from jormungandr import choreo
from AutoSequencerV2.command import Command
from utils.allianceTransformUtils import transform


from Elevatorandmech.ElevatorControl import ElevatorControl, elevDepConstants
from Elevatorandmech.ArmControl import ArmControl, armDepConstants

from positionSchemes.auton_schemes.place_L4_v6_a import PlaceL4V1Auto
from positionSchemes.auton_schemes.drive_fwd_a import DriveFwdAuto
from positionSchemes.auton_schemes.empty_a import EmptyAuto
from wpimath.geometry import Pose2d, Rotation2d
from utils.units import deg2Rad

class AutonPoserSelected(IntEnum):
    B_LEFT_DRIVE = 0
    B_RIGHT_DRIVE = 1
    B_CENTER_DRIVE = 2
    R_LEFT_DRIVE = 3
    R_RIGHT_DRIVE = 4
    R_CENTER_DRIVE = 5
    B_LEFT_REEF = 6
    B_RIGHT_REEF = 7
    R_LEFT_REEF = 8
    R_RIGHT_REEF = 9
    B_LEFT_TO_LEFT_REEF = 10
    B_LEFT_TO_RIGHT_REEF = 11
    B_RIGHT_TO_LEFT_REEF = 12
    B_RIGHT_TO_RIGHT_REEF = 13
    R_LEFT_TO_LEFT_REEF = 14
    R_LEFT_TO_RIGHT_REEF = 15
    R_RIGHT_TO_LEFT_REEF = 16
    R_RIGHT_TO_RIGHT_REEF = 17

class StartingPoses:
    def __init__(self, command):
        self.command = command
        pass
    def getInitialPose(self):
        positions = {
            "BL": Pose2d(7.13, 7.55, Rotation2d(deg2Rad(180))),
            "BC": Pose2d(7.13, 4.031, Rotation2d(deg2Rad(180))),
            "BR": Pose2d(7.13, 0.4826, Rotation2d(deg2Rad(180))),
            "RL": Pose2d(10.418, 0.4826, Rotation2d(deg2Rad(0))),
            "RC": Pose2d(10.418, 4.031, Rotation2d(deg2Rad(0))),
            "RR": Pose2d(10.418, 7.55, Rotation2d(deg2Rad(0))),
        }

        match self.command:
            case AutonPoserSelected.B_LEFT_REEF:
                return positions.get("BC", Pose2d())
            case AutonPoserSelected.B_RIGHT_REEF:
                return positions.get("BC", Pose2d())
            case AutonPoserSelected.R_LEFT_REEF:
                return positions.get("RC", Pose2d())
            case AutonPoserSelected.R_RIGHT_REEF:
                return positions.get("RC", Pose2d())
            case AutonPoserSelected.B_LEFT_TO_LEFT_REEF:
                return positions.get("BL", Pose2d())
            case AutonPoserSelected.B_LEFT_TO_RIGHT_REEF:
                return positions.get("BL", Pose2d())
            case AutonPoserSelected.R_LEFT_TO_LEFT_REEF:
                return positions.get("RL", Pose2d())
            case AutonPoserSelected.R_LEFT_TO_RIGHT_REEF:
                return positions.get("RL", Pose2d())
            case AutonPoserSelected.B_RIGHT_TO_LEFT_REEF:
                return positions.get("BR", Pose2d())
            case AutonPoserSelected.B_RIGHT_TO_RIGHT_REEF:
                return positions.get("BR", Pose2d())
            case AutonPoserSelected.R_RIGHT_TO_LEFT_REEF:
                return positions.get("RR", Pose2d())
            case AutonPoserSelected.R_RIGHT_TO_RIGHT_REEF:
                return positions.get("RR", Pose2d())
            case AutonPoserSelected.B_LEFT_DRIVE:
                return positions.get("BL", Pose2d())
            case AutonPoserSelected.B_RIGHT_DRIVE:
                return positions.get("BR", Pose2d())
            case AutonPoserSelected.B_CENTER_DRIVE:
                return positions.get("BC", Pose2d())
            case AutonPoserSelected.R_LEFT_DRIVE:
                return positions.get("RL", Pose2d())
            case AutonPoserSelected.R_RIGHT_DRIVE:
                return positions.get("RR", Pose2d())
            case AutonPoserSelected.R_CENTER_DRIVE:
                return positions.get("RC", Pose2d())
            case _:
                return Pose2d()


class PoserSchemeCommand(Command):
    def __init__(self, poseScheme):
        self.name = poseScheme
        self.poseScheme = poseScheme
        self.trajCtrl = Trajectory()

        self.done = False

        self.arm = ArmControl()
        self.elev = ElevatorControl()
        self.drivetrain = DrivetrainControl()
        self.poseTelem = self.drivetrain.poseEst._telemetry

        self.poser = 0
        self.path = StartingPoses(poseScheme)


    def initialize(self):
        LEFT = -1
        RIGHT = 1

        BL_TAG = 20
        BC_TAG = 21
        BR_TAG = 22
        RL_TAG = 11
        RC_TAG = 10
        RR_TAG = 9

        BLUE = -1
        RED = 1

        SHORT = 0.25
        LONG = 0.5

        print(f"our pose scheme: {self.poseScheme}")
        match self.poseScheme:
            case AutonPoserSelected.B_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT, BC_TAG)
            case AutonPoserSelected.B_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT, BC_TAG)
            case AutonPoserSelected.R_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT, RC_TAG)
            case AutonPoserSelected.R_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT, RC_TAG)
            case AutonPoserSelected.B_LEFT_TO_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT, BL_TAG)
            case AutonPoserSelected.B_LEFT_TO_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT, BL_TAG)
            case AutonPoserSelected.R_LEFT_TO_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT, RL_TAG)
            case AutonPoserSelected.R_LEFT_TO_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT, RL_TAG)
            case AutonPoserSelected.B_RIGHT_TO_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT, BR_TAG)
            case AutonPoserSelected.B_RIGHT_TO_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT, BR_TAG)
            case AutonPoserSelected.R_RIGHT_TO_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT, RR_TAG)
            case AutonPoserSelected.R_RIGHT_TO_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT, RR_TAG)
            case AutonPoserSelected.B_LEFT_DRIVE:
                self.poser = DriveFwdAuto(self.arm, self.drivetrain, self.elev, BLUE, LONG)
            case AutonPoserSelected.B_RIGHT_DRIVE:
                self.poser = DriveFwdAuto(self.arm, self.drivetrain, self.elev, BLUE, LONG)
            case AutonPoserSelected.B_CENTER_DRIVE:
                self.poser = DriveFwdAuto(self.arm, self.drivetrain, self.elev, BLUE, SHORT)
            case AutonPoserSelected.R_LEFT_DRIVE:
                self.poser = DriveFwdAuto(self.arm, self.drivetrain, self.elev, RED, LONG)
            case AutonPoserSelected.R_RIGHT_DRIVE:
                self.poser = DriveFwdAuto(self.arm, self.drivetrain, self.elev, RED, LONG)
            case AutonPoserSelected.R_CENTER_DRIVE:
                self.poser = DriveFwdAuto(self.arm, self.drivetrain, self.elev, RED, SHORT)
            case _:
                self.poser = EmptyAuto(self.arm, self.drivetrain, self.elev)

    def execute(self):
        self.poser.update()

    def isDone(self):
        return self.done

    def end(self, interrupt):
        self.trajCtrl.setCmdFromChoreoAuton(None)
        self.poseTelem.setChoreoTrajectory(None)

    def getName(self):
        return f"Auton Poser Scheme {self.name}"