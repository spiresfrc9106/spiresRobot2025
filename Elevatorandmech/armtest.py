from utils.singleton import Singleton


class ArmControl(metaclass=Singleton):
    def __init__(self):
        self.hi = 0;
        self.atAboutDown = False

