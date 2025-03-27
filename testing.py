from positionSchemes._posintelligence import PlacementIntelligence
from drivetrain.drivetrainControl import DrivetrainControl
from wpimath.geometry import Pose2d, Rotation2d

#blue side
startingPositionID17 = Pose2d(4.585415363311768, 0.1569528430700302, Rotation2d(0))
startingPositionID18 = Pose2d(2, 4, Rotation2d(0))
startingPositionID19 = Pose2d(3, 6, Rotation2d(0))
startingPositionID20 = Pose2d(6, 6.5, Rotation2d(0))
startingPositionID21 = Pose2d(7, 4, Rotation2d(0))
startingPositionID22 = Pose2d(6, 2, Rotation2d(0))

#red side
startingPositionID11 = Pose2d(11.5, 2, Rotation2d(0))
startingPositionID10 = Pose2d(11, 4, Rotation2d(0))
startingPositionID9 = Pose2d(12, 6, Rotation2d(0))
startingPositionID8 = Pose2d(15, 6, Rotation2d(0))
startingPositionID7 = Pose2d(15, 4, Rotation2d(0))
startingPositionID6 = Pose2d(14, 2, Rotation2d(0))

startPosList = [
startingPositionID17,
startingPositionID18,
startingPositionID19,
startingPositionID20,
startingPositionID21,
startingPositionID22,
startingPositionID11,
startingPositionID10,
startingPositionID9,
startingPositionID8,
startingPositionID7,
startingPositionID6,
]


pI = PlacementIntelligence(DrivetrainControl())

if __name__ == "__main__":
    for pos in startPosList:
        print()
        print()
        print(pI.decidePlacementPoseNoah(1, 0, startingPositionID17))
        print()
        print()


