from enum import Enum
import wpilib
from config import FRC_TEAM_NUMBER
from utils.faults import Fault
from utils.singleton import Singleton

"""
Specifc robots this codebase might run on.
"""
RobotTypes = Enum('RobotTypes', [
    'Main',
    'Practice',
    'TestBoard',
    'Spires2023',
    'Spires2024',
    'SpiresMain',
    'SpiresPractice',
    'SpiresTestBoard',
    'SpiresRoboRioV1'
])

class RobotIdentification(metaclass=Singleton):
    """
    While we strive for our practice robot and main/competition robots to be as identical as possible,
    that's not always the case. 
    The goal of this class is to identify which robot is currently running the code.
    The constants between practice and main robots may be different. 
    """

    def __init__(self):
        self.roboControl = wpilib.RobotController
        self.robotType = None
        self.serialFault = Fault("RoboRIO serial number not recognized")
        self._configureValue()

    def _configureValue(self):

        self.serialFault.setNoFault()

        print(f"self.roboControl.getSerialNumber()={self.roboControl.getSerialNumber()}")

        if self.roboControl.getSerialNumber() == "030e2cb0":
            #Test to see if the RoboRio serial number is the main/"Production" bot.
            self.robotType = RobotTypes.Main 
        elif self.roboControl.getSerialNumber() == "03064e3f" \
                or FRC_TEAM_NUMBER==1736 and wpilib.TimedRobot.isSimulation():
            #Test to see if the RoboRio serial number is the practice bot.
            self.robotType = RobotTypes.Practice
        elif self.roboControl.getSerialNumber() == "0316b37c":
            #Test to see if the RoboRio serial number is our testboard's serial number.
            self.robotType = RobotTypes.TestBoard
        elif self.roboControl.getSerialNumber() == "032430C5" \
                or FRC_TEAM_NUMBER==9106 and wpilib.TimedRobot.isSimulation():
            self.robotType = RobotTypes.Spires2023
        elif self.roboControl.getSerialNumber() == "032B1F4B":
            self.robotType = RobotTypes.Spires2024
        elif self.roboControl.getSerialNumber() == "032B1FBB":
            self.robotType = RobotTypes.SpiresTestBoard
        elif self.roboControl.getSerialNumber() == "03057ab7":
            self.robotType = RobotTypes.SpiresRoboRioV1
        else:
            # If the Robo Rio's serial number is not equal to any of our known serial numbers, 
            # assume we are the main robot. But, throw a fault, since this is something software
            # team needs to fix.
            self.robotType = RobotTypes.SpiresTestBoard
            self.serialFault.setFaulted()
            assert False

    def _getRobotSerialNumber(self)->str:
        return self.roboControl.getSerialNumber()

    def getRobotType(self)->RobotTypes:
        """
        Return which robot we're running on right now
        """
        return self.robotType

    def isSpiresRobot(self)->bool:
        return str(self.robotType).startswith('RobotTypes.Spires')