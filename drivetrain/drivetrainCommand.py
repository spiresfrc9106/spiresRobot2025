from dataclasses import dataclass, field
from wpimath.geometry import Pose2d



@dataclass
class DrivetrainCommand:
    """
    Represents desired drivetrain motion which is currently desired.
    Usually comes from a human driver, but could be from an autonomous momde or assist feature.
    """
    velX:float = 0.0  # Field X velocity in meters/sec
    velY:float = 0.0  # Field Y velocity in meters/secS
    velT:float = 0.0  # Rotational speed in rad/sec
    # pylint: disable=unnecessary-lambda
    desPose:Pose2d = field(default_factory = lambda: Pose2d())  # Current desired pose of the drivetrain
