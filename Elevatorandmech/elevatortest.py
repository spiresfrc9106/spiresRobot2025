from utils.singleton import Singleton


class ElevatorControl(metaclass=Singleton):
    def __init__(self):
        self.atAboutMiddle = False
        self.reachedBottom = False #these only need to be estimates, since the system will continue nmw

