import math
import wpilib
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d
from drivetrain.drivetrainCommand import DrivetrainCommand
from drivetrain.drivetrainPhysical import (
    MAX_FWD_REV_SPEED_MPS,
    MAX_ROTATE_SPEED_RAD_PER_SEC,
)
#from drivetrain.controlStrategies.autoDrive import AutoDrive
from choreo.trajectory import SwerveSample
from utils.calibration import Calibration
from utils.signalLogging import addLog, getNowLogger
from utils.mathUtils import limit
from utils.units import m2in

class HolonomicDriveController:
    """
    Closed-loop controller suite to get the robot from where it is to where it isn't
    https://www.youtube.com/watch?v=bZe5J8SVCYQ
    Used to emulate driver commands while following a trajectory or auto-driving.

    This is often called a "Holonomic Drive Controller" or "HDC".

    Note that wpilib has one of these, but it doesn't (yet) include feed-forward on rotation (??????)
    So we made our own.
    """

    def __init__(self, name:str):
        self.curVx = 0
        self.curVy = 0
        self.curVtheta = 0

        self.transP = Calibration(f"{name} HDC Translation kP", 6.0)
        self.transI = Calibration(f"{name} HDC Translation kI", 0.0)
        self.transD = Calibration(f"{name} HDC Translation kD", 0.0)
        self.rotP = Calibration(f"{name} HDC Rotation kP", 8.0)
        self.rotI = Calibration(f"{name} HDC Rotation kI", 0.0)
        self.rotD = Calibration(f"{name} HDC Rotation kD", .05)

        self.xFF = 0.0
        self.yFF = 0.0
        self.tFF = 0.0
        self.xFB = 0.0
        self.yFB = 0.0
        self.tFB = 0.0

        addLog(f"{name}/HDC/xFF", lambda:self.xFF, "mps")
        addLog(f"{name}/HDC/yFF", lambda:self.yFF, "mps")
        addLog(f"{name}/HDC/tFF", lambda:self.tFF, "radpersec")
        addLog(f"{name}/HDC/xFB", lambda:self.xFB, "mps")
        addLog(f"{name}/HDC/yFB", lambda:self.yFB, "mps")
        addLog(f"{name}/HDC/tFB", lambda:self.tFB, "radpersec")

        self.trajCmdPosField = wpilib.Field2d()
        self.curEstPoseField = wpilib.Field2d()
        wpilib.SmartDashboard.putData(f"{name}/HDC/trajCmdPose", self.trajCmdPosField)
        wpilib.SmartDashboard.putData(f"{name}/HDC/curEstPose", self.curEstPoseField)

        # Closed-loop control for the X position
        self.xCtrl = PIDController(
            self.transP.get(),
            self.transI.get(),
            self.transD.get(),
        )

        # Closed-loop control for the Y position
        self.yCtrl = PIDController(
            self.transP.get(),
            self.transI.get(),
            self.transD.get(),
        )

        # Closed-loop control for rotation (Theta)
        self.tCtrl = PIDController(
            self.rotP.get(),
            self.rotI.get(),
            self.rotD.get(),
        )
        # Make sure the controller knows that -170 and 170 are just 20 degrees apart
        self.tCtrl.enableContinuousInput(-math.pi, math.pi)

    def updateCals(self):
        self.xCtrl.setPID(self.transP.get(), self.transI.get(), self.transD.get())
        self.yCtrl.setPID(self.transP.get(), self.transI.get(), self.transD.get())
        self.tCtrl.setPID(self.rotP.get(), self.rotI.get(), self.rotD.get())

    def update(self, trajCmd: SwerveSample, curEstPose):
        """Main periodic update, call this whenever you need new commands

        Args:
            trajCmd (PathPlannerState): Current trajectory state
            curEstPose (Pose2d): Current best-estimate of where the robot is at on the field

        Returns:
            ChassisSpeeds: the Field-relative set of vx, vy, and vt commands for
            the robot to follow that will get it to the desired pose
        """
        # Feed-Forward - calculate how fast we should be going at this point in the trajectory
        xFF, yFF, tFF = trajCmd.get_chassis_speeds()

        trajCmdPose = trajCmd.get_pose()
        self.trajCmdPosField.setRobotPose(trajCmdPose)
        self.curEstPoseField.setRobotPose(curEstPose)
        return self.update2(xFF,yFF,tFF,trajCmdPose,curEstPose)

    def update2(self, xFF, yFF, tFF, cmdPose:Pose2d, curEstPose:Pose2d):

        #calc some errs
        self.errX_in = m2in(cmdPose.X() - curEstPose.X())
        self.errY_in = m2in(cmdPose.Y() - curEstPose.Y())
        self.errT_deg = (cmdPose.rotation() - curEstPose.rotation()).degrees()

        # Feed-Back - Apply additional correction if we're not quite yet at the spot on the field we
        #             want to be at.
        self.xFB = self.xCtrl.calculate(curEstPose.X(), cmdPose.X())
        self.yFB = self.yCtrl.calculate(curEstPose.Y(), cmdPose.Y())
        self.tFB = self.tCtrl.calculate(
            curEstPose.rotation().radians(), cmdPose.rotation().radians()
        )

        # Remember feed-forward value inputs
        self.xFF = xFF 
        self.yFF = yFF 
        self.tFF = tFF 

        retVal = DrivetrainCommand()
        retVal.velX = limit(xFF + self.xFB, MAX_FWD_REV_SPEED_MPS)
        retVal.velY = limit(yFF + self.yFB, MAX_FWD_REV_SPEED_MPS)
        retVal.velT = limit(tFF + self.tFB, MAX_ROTATE_SPEED_RAD_PER_SEC)
        retVal.desPose = cmdPose

        return retVal
