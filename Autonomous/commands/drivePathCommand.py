import os
import choreo
import choreo.trajectory
from wpilib import Timer
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainControl import DrivetrainControl

from AutoSequencerV2.command import Command
from utils.allianceTransformUtils import transform
from utils.autonomousTransformUtils import flip

#This takes the Choreo file and turns it into a drive path Command

class DrivePathCommand(Command):
    def __init__(self, pathFile, extraAlignTime_s = 0.0):
        self.name = pathFile

        self.trajCtrl = Trajectory()

        # Get the internal path file
        absPath = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "deploy",
                "choreo",
                pathFile,
            )
        )

        self.path = choreo.load_swerve_trajectory(absPath)
        self.done = False
        self.startTime = (
            -1
        )
       
        # we'll populate these for real later, just declare they'll exist
        self.duration = self.path.get_total_time() + extraAlignTime_s
        self.drivetrain = DrivetrainControl()
        self.poseTelem = self.drivetrain.poseEst._telemetry


    def initialize(self):
        self.startTime = Timer.getFPGATimestamp()
        self.poseTelem.setChoreoTrajectory(self.path)
        print(f"DrivePathCommand initialize Noah DrivePathCommand events={self.path.events}")
        for event in self.path.events:
            print(f"DrivePathCommand initialize Noah DrivePathCommand event name={event.event} timestamp={event.timestamp}")


    def execute(self):
        curTime = Timer.getFPGATimestamp() - self.startTime
        curState = self.path.sample_at(curTime)

        if(curState is not None):
            curState = flip(transform(curState))
            self.trajCtrl.setCmdFromChoreoAuton(curState)
        else: 
            self.trajCtrl.setCmdFromChoreoAuton(None)

        if curTime >= self.duration:
            self.trajCtrl.setCmdFromChoreoAuton(None)
            self.poseTelem.setChoreoTrajectory(None)
            self.done = True

    def isDone(self):
        return self.done

    def end(self,interrupt):
        self.trajCtrl.setCmdFromChoreoAuton(None)
        self.poseTelem.setChoreoTrajectory(None)

    def getName(self):
        return f"Drive Trajectory {self.name}"