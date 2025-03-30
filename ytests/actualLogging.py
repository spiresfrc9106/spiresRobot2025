
from utils.signalLogging import addLog
from wpimath.geometry import Pose2d, Rotation2d, Twist2d

class YTestForPositionActual:
    def __init__(self, name):
        self.name = f"ytest_position_{name}"
        self.x_name = self.name + "_x"
        self.y_name = self.name + "_y"
        self.t_name = self.name + "_t"

        self.x_value = 0
        self.y_value = 0
        self.t_value = 0

        addLog(self.x_name, lambda: self.x_value, "")
        addLog(self.y_name, lambda: self.y_value, "")
        addLog(self.t_name, lambda: self.t_value, "")

    def update(self, position: Pose2d):
        self.x_value = position.X()
        self.y_value = position.Y()
        self.t_value = position.rotation().radians()
        pass


