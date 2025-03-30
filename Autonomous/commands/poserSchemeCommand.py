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
from positionSchemes.auton_schemes.empty_a import EmptyAuto
from wpimath.geometry import Pose2d

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
        pass
    def getInitialPose(self):
        return Pose2d(2, 2, 0)

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
        print(f"our pose scheme: {self.poseScheme}")
        match self.poseScheme:
            case AutonPoserSelected.B_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT)
            case AutonPoserSelected.B_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT)
            case AutonPoserSelected.R_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT)
            case AutonPoserSelected.R_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT)
            case AutonPoserSelected.B_LEFT_TO_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT)
            case AutonPoserSelected.B_LEFT_TO_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT)
            case AutonPoserSelected.R_LEFT_TO_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT)
            case AutonPoserSelected.R_LEFT_TO_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT)
            case AutonPoserSelected.B_RIGHT_TO_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT)
            case AutonPoserSelected.B_RIGHT_TO_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT)
            case AutonPoserSelected.R_RIGHT_TO_LEFT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, LEFT)
            case AutonPoserSelected.R_RIGHT_TO_RIGHT_REEF:
                self.poser = PlaceL4V1Auto(self.arm, self.drivetrain, self.elev, RIGHT)
            case _:
                self.poser = EmptyAuto(self.arm, self.drivetrain, self.elev)

    def execute(self):
        print(f"our pose scheme: {self.poseScheme}")
        self.poser.update()

    def isDone(self):
        return self.done

    def end(self, interrupt):
        self.trajCtrl.setCmdFromChoreoAuton(None)
        self.poseTelem.setChoreoTrajectory(None)

    def getName(self):
        return f"Auton Poser Scheme {self.name}"