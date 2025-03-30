from Autonomous.commands.drivePathCommand import DrivePathCommand
from AutoSequencerV2.mode import Mode

from Autonomous.commands.poserSchemeCommand import AutonPoserSelected, PoserSchemeCommand


# Just drives out of the starting zone. That's all.
class BlueRightToReefRight(Mode):
    def __init__(self):
        #this is naming the mode, in this case "Drive Out"
        Mode.__init__(self, f"B-Right To Reef Right")

        #This is setting the path command (pathCmd), which is what we will use. The DrivePathCommand must be 
        #exactly the same as it is in the Choreo name. 
        self.pathCmd = PoserSchemeCommand(AutonPoserSelected.B_LEFT_REEF)

    def getCmdGroup(self):
        # Just return the path command normally, since we're only doing one path. 
        # When changing to the return self.pathCmd, get rid of the pass
        return self.pathCmd

    def getInitialDrivetrainPose(self):
        # Use the path command to specify the starting pose, using getInitialPose()
        return self.pathCmd.path.getInitialPose()
