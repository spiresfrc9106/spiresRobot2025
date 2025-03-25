from drivetrain.controlStrategies.trajectoryGuts import TrajectoryGuts
from utils.singleton import Singleton

class Trajectory(TrajectoryGuts, metaclass=Singleton):
    def __init__(self):
        super().__init__()
        self._name = "Trajectory"

